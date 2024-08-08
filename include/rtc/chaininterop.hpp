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
	// It does not make sense to have 2 different variable types for storing time.
	// But I arrivalTime is calculated from receiver reports and departureTime
	// is captured from system wall clock.
	double arrivalTime;
	std::chrono::microseconds arrivalDuration;
	std::chrono::steady_clock::time_point departureTime;
	PacketInfo(uint16_t numBytes);
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

struct ArrivalGroup {
	std::vector<PacketInfo> packets;
	std::chrono::microseconds arrival_time;
	std::chrono::steady_clock::time_point departure_time;

	ArrivalGroup() = default;
	ArrivalGroup(const ArrivalGroup &other) : packets(other.packets),
		                                      arrival_time(other.arrival_time),
											  departure_time(other.departure_time) {}
	ArrivalGroup(ArrivalGroup&& other) noexcept : packets(std::move(other.packets)),
												  arrival_time(std::move(other.arrival_time)),
												  departure_time(std::move(other.departure_time)) {}
	ArrivalGroup& operator=(const ArrivalGroup& other) {
		if (this != &other) {
			packets = other.packets;
			arrival_time = other.arrival_time;
			departure_time = other.departure_time;
		}
		return *this;
	}
	ArrivalGroup& operator=(ArrivalGroup&& other) noexcept {
		if (this != &other) {
			packets = std::move(other.packets);
			arrival_time = std::move(other.arrival_time);
			departure_time = std::move(other.departure_time);
			other.reset();
		}
		return *this;
	}
	void add(PacketInfo packet) {
		packets.push_back(packet);
		arrival_time = packet.arrivalDuration;
		departure_time = packet.departureTime;
	}

	void reset() {
		packets.clear();
		arrival_time = std::chrono::microseconds::zero();
		departure_time = std::chrono::steady_clock::time_point();
	}
};

class RTC_CPP_EXPORT ChainInterop {
	using clock = std::chrono::steady_clock;
	std::map<uint16_t, PacketInfo> packetInfo;
	std::deque<WholeFrameInfo> wholeFrameInfo;
	// timeThreshold should be at least 1000ms.
	std::chrono::milliseconds timeThreshold;
	std::mutex mapMutex;

public:
	ChainInterop(int thresholdMs);
	void addPackets(uint16_t baseSeqNum, const std::vector<uint16_t> &numBytes);
	void setSentInfo(const std::vector<uint16_t> &seqNums);
	void setSentInfo(const std::vector<uint16_t> &seqNums, const std::vector<clock::time_point> &sendTimes);
	size_t updateReceivedStatus(uint16_t baseSeqNum, const std::vector<bool> &statuses, const std::vector<std::chrono::microseconds> &arrival_times_us);
	BitrateStats getBitrateStats();
	void deleteOldFrames();
	size_t getNumberOfFrames() const;
	size_t getNumberOfFramesReceived() const;
	std::vector<ArrivalGroup> runArrivalGroupAccumulator(uint16_t seqnum, uint16_t num_packets);
};

std::chrono::microseconds interDepartureTimePkt(ArrivalGroup group, PacketInfo packet);
std::chrono::microseconds interArrivalTimePkt(ArrivalGroup group, PacketInfo packet);
std::chrono::microseconds interGroupDelayVariationPkt(ArrivalGroup group, PacketInfo packet);

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* RTC_CHAIN_INTEROP_H */
