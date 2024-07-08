#ifndef RTC_CHAIN_INTEROP_H
#define RTC_CHAIN_INTEROP_H

#if RTC_ENABLE_MEDIA
#include "common.hpp"
#include <map>
#include <deque>
#include <vector>
#include <chrono>
#include <utility>

namespace rtc {

struct ReceivedStats
{
	size_t receivedBytes, notReceivedBytes;
	size_t receivedPackets, notReceivedPackets;

	ReceivedStats(){
		receivedBytes = 0;
		notReceivedBytes = 0;
		receivedPackets = 0;
		notReceivedPackets = 0;
	}

	ReceivedStats operator+(ReceivedStats s){
		ReceivedStats result;
		result.receivedBytes = receivedBytes + s.receivedBytes;
		result.notReceivedBytes = notReceivedBytes + s.notReceivedBytes;
		result.receivedPackets = receivedPackets + s.receivedPackets;
		result.notReceivedPackets = notReceivedPackets + s.notReceivedPackets;

		return result;
	}
};

struct BitrateStats {
	double txBitsPerSecond, rxBitsPerSecond;
	BitrateStats() : txBitsPerSecond(0), rxBitsPerSecond(0) {}
	BitrateStats(double tx, double rx) : txBitsPerSecond(tx), rxBitsPerSecond(rx) {};
};

struct PacketInfo {
	bool isReceived;
	bool isSent;
	uint16_t numBytes;
	uint32_t frameIndex;
	// It does not make sense to have 2 different variable types for storing time.
	// But I arrivalTime is calculated from receiver reports and departureTime
	// is captured from system wall clock.
	double arrivalTime;
	std::chrono::steady_clock::time_point departureTime;
	PacketInfo(uint32_t frameIndex, uint16_t numBytes);
};

class WholeFrameInfo {
	std::chrono::steady_clock::time_point time;
	uint16_t seqNumStart, seqNumEnd;

public:
	WholeFrameInfo(std::chrono::steady_clock::time_point time, uint16_t seqNumStart, uint16_t seqNumEnd);
	std::chrono::steady_clock::time_point getTime() const;
	uint16_t getSeqNumStart() const;
	uint16_t getSeqNumEnd() const;
};

class RTC_CPP_EXPORT ChainInterop {
	std::map<uint16_t, PacketInfo> packetInfo;
	std::deque<WholeFrameInfo> wholeFrameInfo;
	// timeThreshold should be at least 1000ms.
	std::chrono::milliseconds timeThreshold;
	std::mutex mapMutex;
	uint32_t frameCounter;

public:
	ChainInterop(int thresholdMs);
	void addPackets(uint16_t baseSeqNum, std::vector<uint16_t> numBytes);
	void setSentInfo(std::vector<uint16_t> seqNums);
	size_t updateReceivedStatus(uint16_t baseSeqNum, std::vector<bool> statuses, std::vector<double> arrival_times);
	BitrateStats getBitrateStats();
	void deleteOldFrames();
	size_t getNumberOfFrames() const;
	size_t getNumberOfFramesReceived() const;
};
} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* RTC_CHAIN_INTEROP_H */
