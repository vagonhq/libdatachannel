#ifndef RTC_CHAIN_INTEROP_H
#define RTC_CHAIN_INTEROP_H

#if RTC_ENABLE_MEDIA
#include "common.hpp"
#include <map>
#include <vector>
#include <chrono>
#include <utility>

namespace rtc {

struct PacketInfo {
	bool isReceived;
	uint16_t numBytes;
	PacketInfo(bool isReceived, uint16_t numBytes);
};
class FrameInfo {
	std::chrono::steady_clock::time_point time;
	std::vector<PacketInfo> packets;

public:
	FrameInfo(std::chrono::steady_clock::time_point time);
	void addPacket(uint16_t numBytes);
	size_t updateReceivedStatus(std::vector<bool> statuses, size_t packetStartIdx, size_t statusStartIdx);
	size_t size() const;
	size_t getFrameSizeInBytes() const;
	std::chrono::steady_clock::time_point getTime() const;
};

class RTC_CPP_EXPORT ChainInterop {
	std::map<uint16_t, FrameInfo> outgoingFrameInfo;
	// I give a little bit of leeway to one second threshold since the elapsed time
	// is generally between 0.98 and 0.99
	const std::chrono::milliseconds oneSecond = std::chrono::milliseconds(1010);

public:
	ChainInterop();
	void addFrame(uint16_t seqNum);
	void addPacketToFrame(uint16_t seqNum, uint16_t numBytes);
	void updateReceivedStatus(uint16_t baseSeqNum, std::vector<bool> statuses);
	double getReceivedBitsPerSecond();
	void deleteOldFrames(std::chrono::steady_clock::time_point time_now);
	size_t size() const;
};
} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* RTC_CHAIN_INTEROP_H */
