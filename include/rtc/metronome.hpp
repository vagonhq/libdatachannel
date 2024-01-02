#ifndef PACER_H
#define PACER_H

#if RTC_ENABLE_MEDIA

#include <deque>
#include <chrono>
#include "mediahandler.hpp"
#include "chaininterop.hpp"

namespace rtc {

class RTC_CPP_EXPORT PacerAlgorithm {
public:
	virtual unsigned int getBudget() = 0;
	virtual unsigned int getPace() = 0;
	virtual void setPace(unsigned int pace) = 0;
	virtual void setBudget(unsigned int budget) = 0;
	virtual void resetBudget() = 0;
};

class RTC_CPP_EXPORT Metronome final : public MediaHandler {
    using clock = std::chrono::steady_clock;

    std::deque<message_ptr> send_queue;
	size_t mQueueSizeInBytes;
	size_t mMaxQueueSizeInBytes;
    std::mutex send_queue_mutex;

	clock::time_point prev;
	std::chrono::milliseconds mThreadDelay;
	std::function<void(message_vector&)> mProcessPacketsCallback; 
	std::shared_ptr<PacerAlgorithm> mPacerAlgorithm;

public:
	Metronome(size_t maxQueueSizeInBytes, std::shared_ptr<PacerAlgorithm> pacerAlgorithm,
	          std::function<void(message_vector &)> processPacketsCallback);
	
	void outgoing(message_vector &messages, const message_callback &send) override;
	void senderProcess(const message_callback &send);

};

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* PACER_H */
