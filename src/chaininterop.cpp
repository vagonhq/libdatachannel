#if RTC_ENABLE_MEDIA

#include "chaininterop.hpp"

namespace rtc {


PacketInfo::PacketInfo(bool isReceived, uint16_t numBytes) : isReceived(isReceived), numBytes(numBytes) {}

FrameInfo::FrameInfo(std::chrono::steady_clock::time_point time) : time(time) {}

void FrameInfo::addPacket(uint16_t numBytes) { packets.emplace_back(std::move(PacketInfo(false, numBytes))); }

size_t FrameInfo::updateReceivedStatus(std::vector<bool> statuses, size_t packetStartIdx, size_t statusStartIdx) {
	unsigned int bound = packets.size() - packetStartIdx < statuses.size() - statusStartIdx
	                         ? packets.size() - packetStartIdx : statuses.size() - statusStartIdx;
	for (unsigned int i = 0; i < bound; i++) {
		packets.at(i + packetStartIdx).isReceived |= statuses.at(statusStartIdx + i);
	}
	return bound;
}

size_t FrameInfo::size() const { return packets.size(); }

ReceivedStats FrameInfo::getFrameSizeInBytes() const {
	ReceivedStats stats = ReceivedStats();
	for (auto packet : packets) {
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
	for (auto packet : packets) {
		result = result & packet.isReceived;
	}
	return result;
}

ChainInterop::ChainInterop(int thresholdMs) { timeThreshold = std::chrono::milliseconds(thresholdMs); }

void ChainInterop::addFrame(uint16_t seqNum) {
	auto time_now = std::chrono::steady_clock::now();
	outgoingFrameInfo.emplace(std::make_pair(seqNum, time_now));
}

void ChainInterop::addPacketToFrame(uint16_t seqNum, uint16_t numBytes) { outgoingFrameInfo.at(seqNum).addPacket(numBytes); }

size_t ChainInterop::updateReceivedStatus(uint16_t baseSeqNum, std::vector<bool> statuses) {
	size_t statusStartIdx = 0;
	uint16_t seqNum = baseSeqNum;
	size_t processedStatusCount = 0;
	size_t totalProcessedStatusCount = 0;

	if (!outgoingFrameInfo.empty()) {
		while (statusStartIdx < statuses.size()) {
			if (outgoingFrameInfo.count(seqNum)) {
				processedStatusCount =
				    outgoingFrameInfo.at(seqNum).updateReceivedStatus(statuses, 0, statusStartIdx);
				seqNum += processedStatusCount;
				totalProcessedStatusCount += processedStatusCount;
			} else {
				auto iterator = outgoingFrameInfo.lower_bound(seqNum);
				if (iterator != outgoingFrameInfo.end()) {
					iterator--;
					if (outgoingFrameInfo.count(iterator->first))
						processedStatusCount = iterator->second.updateReceivedStatus(
						    statuses, seqNum - iterator->first, statusStartIdx);
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

double ChainInterop::getReceivedBitsPerSecond() {
	if (outgoingFrameInfo.empty())
		return 0;

	std::chrono::steady_clock::time_point timeNow = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point firstPacketTime = timeNow;
	deleteOldFrames(timeNow);
	ReceivedStats allStats;
	for (auto it = outgoingFrameInfo.begin(); it != outgoingFrameInfo.end(); it++) {
		if (it->second.getTime() < firstPacketTime) {
			firstPacketTime = it->second.getTime();
		}
		ReceivedStats temp = it->second.getFrameSizeInBytes();
		allStats = allStats + temp;
	}

	double elapsedSeconds =
	    (double)std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - firstPacketTime).count() / 1000.0;
	double receivedBitrate = (double)allStats.receivedBytes * 8 / elapsedSeconds;

	return receivedBitrate;
}

void ChainInterop::deleteOldFrames(std::chrono::steady_clock::time_point time_now) {
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
} // namespace rtc

#endif
