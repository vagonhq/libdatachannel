#if RTC_ENABLE_MEDIA

#include "metronome.hpp"
#include "impl/threadpool.hpp"
#include "rtp.hpp"

namespace rtc {

using ThreadPool = rtc::impl::ThreadPool;

Metronome::Metronome(unsigned int initialPaceInBytes,
                     std::function<void(message_vector &)> processPacketsCallback,
                     std::function<unsigned int(void)> getPaceInBytesCallback)
    : mPaceInBytes(initialPaceInBytes), mBudget(initialPaceInBytes), mThreadDelay(5),
      mProcessPacketsCallback(processPacketsCallback), mGetPaceInBytesCallback(getPaceInBytesCallback) {
	prev = clock::now();
	queue_size = 0;
}

void Metronome::outgoing(message_vector &messages, const message_callback &send) {
	message_vector others;
	std::unique_lock<std::mutex> lock(send_queue_mutex);
	for (auto &message : messages) {
		if (message->type != Message::Binary) {
			others.push_back(std::move(message));
			continue;
		}
		queue_size += message->size();
		send_queue.push_back(std::move(message));
	}
	while (queue_size > mPaceInBytes) {
		size_t msg_size = send_queue.front()->size();
		// I am not dropping a packet that is just over the limit.
		if (mPaceInBytes > queue_size - msg_size)
			break;
		queue_size -= msg_size;
		send_queue.pop_front();
	}
	messages.swap(others);
	ThreadPool::Instance().schedule(mThreadDelay, [this, &send]() { senderProcess(send); });
}

void Metronome::senderProcess(const message_callback &send) {
	auto now = clock::now();
	if (mGetPaceInBytesCallback) {
		auto newPace = mGetPaceInBytesCallback();
		if (newPace != mPaceInBytes) {
			prev = now;
			mPaceInBytes = newPace;
			mBudget = mPaceInBytes;
		}
	}

	if (now - prev > std::chrono::seconds(1)) {
		mBudget = mPaceInBytes;
		prev = now;
	}

	if (!send_queue.empty()) {
		message_vector outgoing;
		{
			std::unique_lock<std::mutex> lock(send_queue_mutex);
			while (!send_queue.empty() && mBudget >= send_queue.front()->size()) {
				size_t msg_size = send_queue.front()->size(); 
				mBudget -= msg_size;
				queue_size -= msg_size;
				outgoing.push_back(std::move(send_queue.front()));
				send_queue.pop_front();
			}
		}
		for (const auto &message : outgoing) {
			send(message);
		}
		if (mProcessPacketsCallback) {
			mProcessPacketsCallback(outgoing);
		}
		if (!send_queue.empty()) {
			ThreadPool::Instance().schedule(mThreadDelay, [this, &send]() { senderProcess(send); });
		}
	}
}

}

#endif
