#ifndef PACER_H
#define PACER_H

#if RTC_ENABLE_MEDIA

#include <deque>
#include <chrono>
#include "mediahandler.hpp"
#include "chaininterop.hpp"

namespace rtc {
	
class RTC_CPP_EXPORT Metronome final : public MediaHandler {
    using clock = std::chrono::steady_clock;

	unsigned int mPaceInBytes;
	unsigned int mBudget;
    std::deque<message_ptr> send_queue;
	size_t queue_size;
    std::mutex send_queue_mutex;

	clock::time_point prev;
	std::chrono::milliseconds mThreadDelay;
	std::function<void(message_vector&)> mProcessPacketsCallback; 
	std::function<unsigned int(void)> mGetPaceInBytesCallback;

public:
	Metronome(unsigned int initialPaceInBytes, std::function<void(message_vector &)> processPacketsCallback,
	          std::function<unsigned int(void)> getPaceInBytesCallback);
	
	void outgoing(message_vector &messages, const message_callback &send) override;
	void senderProcess(const message_callback &send);

};

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* PACER_H */
