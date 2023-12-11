#if RTC_ENABLE_MEDIA

#include "chaininterop.hpp"
#include <algorithm>
#include <cmath>

namespace rtc {

PacketInfo::PacketInfo(bool isReceived, uint16_t numBytes) : isReceived(isReceived), numBytes(numBytes) { arrival_time = 0; }

FrameInfo::FrameInfo(std::chrono::steady_clock::time_point time) : time(time) {}

void FrameInfo::addPacket(uint16_t numBytes) { packets.emplace_back(std::move(PacketInfo(false, numBytes))); }

size_t FrameInfo::updateReceivedStatus(std::vector<bool> statuses, std::vector<double> arrival_times, size_t packetStartIdx, size_t statusStartIdx) {
	unsigned int bound = packets.size() - packetStartIdx < statuses.size() - statusStartIdx
	                         ? packets.size() - packetStartIdx : statuses.size() - statusStartIdx;
	for (unsigned int i = 0; i < bound; i++) {
		packets.at(i + packetStartIdx).isReceived |= statuses.at(statusStartIdx + i);
		packets.at(i + packetStartIdx).arrival_time = arrival_times.at(statusStartIdx + i);
	}
	return bound;
}

size_t FrameInfo::size() const { return packets.size(); }

ReceivedStats FrameInfo::getFrameSizeInBytes() const {
	ReceivedStats stats = ReceivedStats();
	for (const auto& packet : packets) {
		if (packet.isReceived){
			stats.receivedBytes += packet.numBytes;
			stats.receivedPackets++;
		}
		else
		{
			stats.notReceivedBytes += packet.numBytes;
			stats.notReceivedPackets++;
		}
	}
	return stats;
}

std::chrono::steady_clock::time_point FrameInfo::getTime() const { return time; }

bool FrameInfo::isFullyReceived() const {
	bool result = true;
	for (const auto& packet : packets) {
		result = result && packet.isReceived;
	}
	return result;
}

double FrameInfo::findLastArrivalTime() const {
	auto result = std::max_element(packets.cbegin(), packets.cend(),
	                               [](const PacketInfo &a, const PacketInfo &b) -> bool {
		                               return a.arrival_time < b.arrival_time;
	                               });
	return result->arrival_time;
}

std::chrono::steady_clock::time_point FrameInfo::getDepartureTime() const { return time; }

ChainInterop::ChainInterop(int thresholdMs) { timeThreshold = std::chrono::milliseconds(thresholdMs); }

void ChainInterop::addFrame(uint16_t seqNum) {
	auto time_now = std::chrono::steady_clock::now();
	outgoingFrameInfo.emplace(std::make_pair(seqNum, time_now));
	deleteOldFrames();
}

void ChainInterop::addPacketToFrame(uint16_t seqNum, uint16_t numBytes) { outgoingFrameInfo.at(seqNum).addPacket(numBytes); }

size_t ChainInterop::updateReceivedStatus(uint16_t baseSeqNum, std::vector<bool> statuses, std::vector<double> arrival_times) {
	size_t statusStartIdx = 0;
	uint16_t seqNum = baseSeqNum;
	size_t processedStatusCount = 0;
	size_t totalProcessedStatusCount = 0;

	if (!outgoingFrameInfo.empty()) {
		while (statusStartIdx < statuses.size()) {
			if (outgoingFrameInfo.count(seqNum)) {
				processedStatusCount = outgoingFrameInfo.at(seqNum).updateReceivedStatus(statuses, arrival_times, 0, statusStartIdx);
				seqNum += processedStatusCount;
				totalProcessedStatusCount += processedStatusCount;
			} else {
				auto iterator = outgoingFrameInfo.lower_bound(seqNum);
				if (iterator != outgoingFrameInfo.end()) {
					iterator--;
					if (outgoingFrameInfo.count(iterator->first))
						processedStatusCount = iterator->second.updateReceivedStatus(statuses, arrival_times, seqNum - iterator->first, statusStartIdx);
					else
						break;
					seqNum += processedStatusCount;
					totalProcessedStatusCount += processedStatusCount;
				} else {
					break;
				}
			}
			statusStartIdx += processedStatusCount;
		}
	}
	return totalProcessedStatusCount;
}

BitrateStats ChainInterop::getBitrateStats() {
	if (outgoingFrameInfo.empty())
		return BitrateStats();

	const std::chrono::seconds oneSecond = std::chrono::seconds(1);
	std::chrono::steady_clock::time_point timeNow = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point firstPacketTime = timeNow;

	ReceivedStats allStats;
	for (auto it = outgoingFrameInfo.begin(); it != outgoingFrameInfo.end(); it++) {
		std::chrono::steady_clock::time_point frameTime = it->second.getTime();
		if (frameTime - timeNow < oneSecond) {
			if (frameTime < firstPacketTime) {
				firstPacketTime = frameTime;
			}
			ReceivedStats temp = it->second.getFrameSizeInBytes();
			allStats = allStats + temp;
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
	std::chrono::steady_clock::time_point time_now = std::chrono::steady_clock::now();
	auto it = outgoingFrameInfo.cbegin();
	while (it != outgoingFrameInfo.cend()) {
		if (time_now - it->second.getTime() > timeThreshold)
			it = outgoingFrameInfo.erase(it);
		else
			it++;
	}
}

size_t ChainInterop::size() const { return outgoingFrameInfo.size(); }

size_t ChainInterop::sizeReceived() const {
	size_t nReceived = 0;
	for (auto it = outgoingFrameInfo.begin(); it != outgoingFrameInfo.end(); it++) {
		nReceived += it->second.isFullyReceived() ? 1 : 0;
	}
	return nReceived;
}

long long ChainInterop::findArrivalIntervalLastTwoFramesMS() const {
	long long result = 0;
	auto last_sent = std::max_element(outgoingFrameInfo.begin(), outgoingFrameInfo.end(),
	                 [](const std::pair<uint16_t, FrameInfo> &a, const std::pair<uint16_t, FrameInfo> &b) -> bool {
		                 return a.second.getDepartureTime() < b.second.getDepartureTime();
	                 });
	if (last_sent != outgoingFrameInfo.begin()) {
		auto prev = std::prev(last_sent);
		result = std::chrono::duration_cast<std::chrono::milliseconds>(last_sent->second.getTime() - prev->second.getTime()).count();
	} else {
		auto prev = outgoingFrameInfo.rend();
		result = std::chrono::duration_cast<std::chrono::milliseconds>(last_sent->second.getTime() - prev->second.getTime()).count();
	}

	return result;
}
} // namespace rtc

#endif
