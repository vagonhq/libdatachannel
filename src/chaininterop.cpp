#if RTC_ENABLE_MEDIA

#include "chaininterop.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
namespace rtc {

PacketInfo::PacketInfo(uint32_t frameIndex, uint16_t numBytes)
    : frameIndex(frameIndex), numBytes(numBytes), isReceived(false), isSent(false) {}

WholeFrameInfo::WholeFrameInfo(std::chrono::steady_clock::time_point time, uint16_t seqNumStart, uint16_t seqNumEnd) 
	: time(time), seqNumStart(seqNumStart), seqNumEnd(seqNumEnd) {}

std::chrono::steady_clock::time_point WholeFrameInfo::getTime() const { return time; }

uint16_t WholeFrameInfo::getSeqNumStart() const { return seqNumStart; }

uint16_t WholeFrameInfo::getSeqNumEnd() const { return seqNumEnd; }

ChainInterop::ChainInterop(int thresholdMs) {
	timeThreshold = std::chrono::milliseconds(thresholdMs);
}

void ChainInterop::addPackets(uint16_t baseSeqNum, std::vector<uint16_t> numBytes) {
	std::unique_lock<std::mutex> guard(mapMutex);
	wholeFrameInfo.emplace_back(std::chrono::steady_clock::now(), baseSeqNum, baseSeqNum + numBytes.size());
	for (const auto& bytes : numBytes) {
		packetInfo.emplace(std::make_pair(baseSeqNum, std::move(PacketInfo(frameCounter, bytes))));
		baseSeqNum++;
	}
	frameCounter++;
}

void ChainInterop::setSentInfo(std::vector<uint16_t> seqNums) {
	std::unique_lock<std::mutex> guard(mapMutex);
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	for (const auto &seqNum : seqNums) {
		packetInfo.at(seqNum).isSent = true;
		packetInfo.at(seqNum).departureTime = now;
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
	std::chrono::steady_clock::time_point timeNow = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point firstPacketTime = timeNow;

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
	std::cout << "received " << allStats.receivedPackets << " not recv "
	          << allStats.notReceivedPackets << std::endl;
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
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

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
	for (const auto &frame : wholeFrameInfo) {
		uint16_t seqNum = frame.getSeqNumStart();
		uint16_t seqNumEnd = frame.getSeqNumEnd();
		while (seqNum < seqNumEnd) {
			nReceived += packetInfo.at(seqNum).isReceived ? 1 : 0;
			seqNum++;
		}
	}
	
	return nReceived;
}

} // namespace rtc

#endif
