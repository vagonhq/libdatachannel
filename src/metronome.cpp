#if RTC_ENABLE_MEDIA

#include "metronome.hpp"
#include "impl/threadpool.hpp"
#include "rtp.hpp"

namespace rtc {

using ThreadPool = rtc::impl::ThreadPool;

Metronome::Metronome(size_t maxQueueSizeInBytes, std::shared_ptr<PacerAlgorithm> pacer,
                     std::function<void(message_vector &)> processPacketsCallback)
    : mMaxQueueSizeInBytes(maxQueueSizeInBytes), mPacerAlgorithm(pacer),
      mProcessPacketsCallback(processPacketsCallback),
      mThreadDelay(5) {
	prev = clock::now();
	mQueueSizeInBytes = 0;
}

void Metronome::outgoing(message_vector &messages, const message_callback &send) {
	message_vector others;
	std::unique_lock<std::mutex> lock(send_queue_mutex);
	for (auto &message : messages) {
		if (message->type != Message::Binary) {
			others.push_back(std::move(message));
			continue;
		}
		mQueueSizeInBytes += message->size();
		send_queue.push_back(std::move(message));
	}
	while (mQueueSizeInBytes > mMaxQueueSizeInBytes) {
		size_t msg_size = send_queue.front()->size();
		// I am not dropping a packet that is just over the limit.
		if (mMaxQueueSizeInBytes > mQueueSizeInBytes - msg_size)
			break;
		mQueueSizeInBytes -= msg_size;
		send_queue.pop_front();
	}
	messages.swap(others);
	ThreadPool::Instance().schedule(mThreadDelay, [this, &send]() { senderProcess(send); });
}

void Metronome::senderProcess(const message_callback &send) {
	auto now = clock::now();

	if (!send_queue.empty()) {
		unsigned int budget = mPacerAlgorithm->getBudget();
		message_vector outgoing;
		{
			std::unique_lock<std::mutex> lock(send_queue_mutex);
			while (!send_queue.empty() && budget >= send_queue.front()->size()) {
				size_t msg_size = send_queue.front()->size(); 
				budget -= msg_size;
				mQueueSizeInBytes -= msg_size;
				outgoing.push_back(std::move(send_queue.front()));
				send_queue.pop_front();
			}
		}
		mPacerAlgorithm->setBudget(budget);
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
