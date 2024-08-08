#if RTC_ENABLE_MEDIA

#include "chaininterop.hpp"
#include <algorithm>
#include <cmath>

namespace rtc {

PacketInfo::PacketInfo(uint16_t numBytes)
    : numBytes(numBytes), isReceived(false), isSent(false) {}

WholeFrameInfo::WholeFrameInfo(std::chrono::steady_clock::time_point time, uint16_t seqNumStart, uint16_t seqNumEnd) 
	: time(time), seqNumStart(seqNumStart), seqNumEnd(seqNumEnd) {}

std::chrono::steady_clock::time_point WholeFrameInfo::getTime() const { return time; }

uint16_t WholeFrameInfo::getSeqNumStart() const { return seqNumStart; }

uint16_t WholeFrameInfo::getSeqNumEnd() const { return seqNumEnd; }

ChainInterop::ChainInterop(int thresholdMs) {
	timeThreshold = std::chrono::milliseconds(thresholdMs);
}

void ChainInterop::addPackets(uint16_t baseSeqNum, const std::vector<uint16_t> &numBytes) {
	std::unique_lock<std::mutex> guard(mapMutex);
	wholeFrameInfo.emplace_back(clock::now(), baseSeqNum, baseSeqNum + numBytes.size());
	for (const auto& bytes : numBytes) {
		packetInfo.emplace(std::make_pair(baseSeqNum, std::move(PacketInfo(bytes))));
		baseSeqNum++;
	}
}

void ChainInterop::setSentInfo(const std::vector<uint16_t> &seqNums) {
	std::unique_lock<std::mutex> guard(mapMutex);
	clock::time_point now = std::chrono::steady_clock::now();
	for (const auto &seqNum : seqNums) {
		packetInfo.at(seqNum).isSent = true;
		packetInfo.at(seqNum).departureTime = now;
	}
}

void ChainInterop::setSentInfo(const std::vector<uint16_t> &seqNums,
                               const std::vector<clock::time_point> &sendTimes) {
	std::unique_lock<std::mutex> guard(mapMutex);
	for (size_t i = 0; i < seqNums.size(); i++) {
		packetInfo.at(seqNums.at(i)).isSent = true;
		packetInfo.at(seqNums.at(i)).departureTime = sendTimes.at(i);
	}
}

size_t ChainInterop::updateReceivedStatus(uint16_t baseSeqNum, const std::vector<bool> &statuses, const std::vector<std::chrono::microseconds> &arrival_times_us) {
	size_t statusStartIdx = 0;
	uint16_t seqNum = baseSeqNum;
	size_t totalProcessedStatusCount = 0;

	if (!packetInfo.empty()) {
		size_t vectorIdx = 0;
		while (vectorIdx < statuses.size()) {
			auto it = packetInfo.find(seqNum);
			if (it != packetInfo.end()) {
				it->second.isReceived = statuses.at(vectorIdx);
				it->second.arrivalDuration = arrival_times_us.at(vectorIdx);
				totalProcessedStatusCount++;
			}
			vectorIdx++;
			seqNum++;
		}
	}
	return totalProcessedStatusCount;
}

BitrateStats ChainInterop::getBitrateStats() {
	if (packetInfo.empty())
		return BitrateStats();
	std::unique_lock<std::mutex> guard(mapMutex);
	const std::chrono::seconds oneSecond = std::chrono::seconds(1);
	clock::time_point timeNow = std::chrono::steady_clock::now();
	clock::time_point firstPacketTime = timeNow;

	ReceivedStats allStats;
	for (const auto &packet : packetInfo) {
		if (packet.second.isSent) {
			if (timeNow - packet.second.departureTime < oneSecond) {
				if (packet.second.departureTime < firstPacketTime) {
					firstPacketTime = packet.second.departureTime;
				}
				if (packet.second.isReceived) {
					allStats.receivedBytes += packet.second.numBytes;
					allStats.receivedPackets++;
				} else {
					allStats.notReceivedBytes += packet.second.numBytes;
					allStats.notReceivedPackets++;
				}
				
			}
		}
	}
	double elapsedSeconds =
	    (double)std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - firstPacketTime).count() / 1000.0;
	double receivedBitrate = (double)allStats.receivedBytes * 8 / elapsedSeconds;
	double sentBitrate = (double)(allStats.receivedBytes + allStats.notReceivedBytes) * 8 / elapsedSeconds;
	if (!isfinite(receivedBitrate))
		receivedBitrate = 0;
	if (!isfinite(sentBitrate))
		sentBitrate = 0;
	return BitrateStats(sentBitrate, receivedBitrate);
}

void ChainInterop::deleteOldFrames() {
	std::unique_lock<std::mutex> guard(mapMutex);
	clock::time_point now = std::chrono::steady_clock::now();

	while (!wholeFrameInfo.empty() && now - wholeFrameInfo.front().getTime() > timeThreshold) {
		uint16_t seqNum = wholeFrameInfo.front().getSeqNumStart();
		uint16_t seqNumEnd = wholeFrameInfo.front().getSeqNumEnd();
		// Due to wraparound seqNum might be greater than seqNumEnd
		while (seqNum != seqNumEnd) {
			packetInfo.erase(seqNum);
			seqNum++;
		}
		wholeFrameInfo.pop_front();
	}
}

size_t ChainInterop::getNumberOfFrames() const { return wholeFrameInfo.size(); }

size_t ChainInterop::getNumberOfFramesReceived() const {
	size_t nReceived = 0;
	bool frameReceived = true;
	for (const auto &frame : wholeFrameInfo) {
		uint16_t seqNum = frame.getSeqNumStart();
		uint16_t seqNumEnd = frame.getSeqNumEnd();
		while (seqNum < seqNumEnd) {
			frameReceived &= packetInfo.at(seqNum).isReceived;
			seqNum++;
		}
		if (frameReceived) {
			nReceived++;
		}
		frameReceived = true;
	}
	
	return nReceived;
}

std::vector<ArrivalGroup> ChainInterop::runArrivalGroupAccumulator(uint16_t seqnum, uint16_t num_packets) {
	const std::chrono::microseconds inter_departure_threshold(5000);
	const std::chrono::microseconds inter_arrival_threshold(5000);
	const std::chrono::microseconds inter_group_delay_variation_threshold(0);

	bool init = false;
	ArrivalGroup group;
	std::vector<ArrivalGroup> groups;
	std::unique_lock<std::mutex> guard(mapMutex);
	for (size_t i = 0; i < num_packets; i++, seqnum++) {
		if (!init) {
			group.add(packetInfo.at(seqnum));
			init = true;
			continue;
		}
		if (packetInfo.at(seqnum).arrivalDuration < group.arrival_time) {
			// ignores out of order arrivals
			continue;
		}
		if (packetInfo.at(seqnum).departureTime > group.departure_time) {
			if (interDepartureTimePkt(group, packetInfo.at(seqnum)) <= inter_departure_threshold) {
				group.add(packetInfo.at(seqnum));
				continue;
			}
			if (interArrivalTimePkt(group, packetInfo.at(seqnum)) <= inter_arrival_threshold &&
			    interGroupDelayVariationPkt(group, packetInfo.at(seqnum)) <
			        inter_group_delay_variation_threshold) {
				group.add(packetInfo.at(seqnum));
				continue;
			}
			groups.push_back(std::move(group));
			group.reset();
			group.add(packetInfo.at(seqnum));
		}
	}

	return groups;
}

std::chrono::microseconds interDepartureTimePkt(ArrivalGroup group, PacketInfo packet) {
	if (group.packets.empty())
		return std::chrono::microseconds::zero();

	return std::chrono::duration_cast<std::chrono::microseconds>(
	    packet.departureTime - group.packets.back().departureTime);
}
std::chrono::microseconds interArrivalTimePkt(ArrivalGroup group, PacketInfo packet) {
	return packet.arrivalDuration - group.packets.back().arrivalDuration;
}
std::chrono::microseconds interGroupDelayVariationPkt(ArrivalGroup group, PacketInfo packet) {
	auto inter_departure = std::chrono::duration_cast<std::chrono::microseconds>(
	    packet.departureTime - group.departure_time);
	auto inter_arrival = packet.arrivalDuration - group.arrival_time;
	return inter_arrival - inter_departure;
}

} // namespace rtc

#endif
