#if RTC_ENABLE_MEDIA

#include "chaininterop.hpp"
#include <algorithm>
#include <cmath>

namespace rtc {

PacketInfo::PacketInfo(uint16_t numBytes)
    : numBytes(numBytes), isReceived(false), isSent(false) {}

FrameInfo::FrameInfo(std::chrono::steady_clock::time_point time, uint16_t seqNumStart, uint16_t seqNumEnd) 
	: time(time), seqNumStart(seqNumStart), seqNumEnd(seqNumEnd) {}

std::chrono::steady_clock::time_point FrameInfo::getTime() const { return time; }

uint16_t FrameInfo::getSeqNumStart() const { return seqNumStart; }

uint16_t FrameInfo::getSeqNumEnd() const { return seqNumEnd; }

ChainInterop::ChainInterop(int thresholdMs) {
	timeThreshold = std::chrono::milliseconds(thresholdMs);
}

void ChainInterop::addPackets(uint16_t baseSeqNum, std::vector<uint16_t> numBytes) {
	std::unique_lock<std::mutex> guard(mapMutex);
	frameInfo.emplace_back(clock::now(), baseSeqNum, baseSeqNum + numBytes.size());
	for (const auto& bytes : numBytes) {
		packetInfo.emplace(std::make_pair(baseSeqNum, std::move(PacketInfo(bytes))));
		baseSeqNum++;
	}
}

void ChainInterop::setSentInfo(std::vector<uint16_t> seqNums) {
	std::unique_lock<std::mutex> guard(mapMutex);
	clock::time_point now = clock::now();
	for (const auto &seqNum : seqNums) {
		packetInfo.at(seqNum).isSent = true;
		packetInfo.at(seqNum).departureTime = now;
	}
}

void ChainInterop::setSentInfo(std::vector<uint16_t> seqNums, std::vector<clock::time_point> sendTimes) {
	std::unique_lock<std::mutex> guard(mapMutex);
	for (size_t i = 0; i < seqNums.size(); i++) {
		packetInfo.at(seqNums.at(i)).isSent = true;
		packetInfo.at(seqNums.at(i)).departureTime = sendTimes.at(i);
	}
}

size_t ChainInterop::updateReceivedStatus(uint16_t baseSeqNum, std::vector<bool> statuses, std::vector<double> arrival_times) {
	size_t statusStartIdx = 0;
	uint16_t seqNum = baseSeqNum;
	size_t totalProcessedStatusCount = 0;

	if (!packetInfo.empty()) {
		size_t vectorIdx = 0;
		while (vectorIdx < statuses.size()) {
			auto it = packetInfo.find(seqNum);
			if (it != packetInfo.end()) {
				it->second.isReceived = statuses.at(vectorIdx);
				it->second.arrivalTime = arrival_times.at(vectorIdx);
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
	clock::time_point timeNow = clock::now();
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
	clock::time_point now = clock::now();

	while (!frameInfo.empty() && now - frameInfo.front().getTime() > timeThreshold) {
		uint16_t seqNum = frameInfo.front().getSeqNumStart();
		uint16_t seqNumEnd = frameInfo.front().getSeqNumEnd();
		// Due to wraparound seqNum might be greater than seqNumEnd
		while (seqNum != seqNumEnd) {
			packetInfo.erase(seqNum);
			seqNum++;
		}
		frameInfo.pop_front();
	}
}

size_t ChainInterop::getNumberOfFrames() const { return frameInfo.size(); }

size_t ChainInterop::getNumberOfFramesReceived() const {
	size_t nReceived = 0;
	bool frameReceived = true;
	for (const auto &frame : frameInfo) {
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

} // namespace rtc

#endif
