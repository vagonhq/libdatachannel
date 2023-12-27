#if RTC_ENABLE_MEDIA

#include "metronome.hpp"
#include "impl/threadpool.hpp"
#include <iostream>
namespace rtc {

using ThreadPool = rtc::impl::ThreadPool;

Metronome::Metronome(unsigned int initialPaceInBytes, std::shared_ptr<ChainInterop> interop)
    : mPaceInBytes(initialPaceInBytes), mBudget(initialPaceInBytes), mThreadDelay(5) {
	prev = clock::now();
	queue_size = 0;
	twccInterop = interop;
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
	if (now - prev > std::chrono::seconds(1)) {
		mBudget = mPaceInBytes;
		prev = now;
	}

	if (!send_queue.empty()) {
		std::unique_lock<std::mutex> lock(send_queue_mutex);

		std::vector<uint16_t> seqNums;
		std::vector<clock::time_point> sendTimes;
		while (!send_queue.empty() && mBudget >= send_queue.front()->size()) {
			size_t msg_size = send_queue.front()->size(); 
			mBudget -= msg_size;
			queue_size -= msg_size;
			auto rtpHeader = reinterpret_cast<RtpHeader *>(send_queue.front()->data());
			auto twccHeader = reinterpret_cast<RtpTwccExt *>(rtpHeader->getExtensionHeader());
			seqNums.push_back(twccHeader->getTwccSeqNum());
			sendTimes.push_back(clock::now());
			send(send_queue.front());
			send_queue.pop_front();
		}
		if (twccInterop) {
			twccInterop->setSentInfo(seqNums, sendTimes);
		}

		if (!send_queue.empty()) {
			ThreadPool::Instance().schedule(mThreadDelay, [this, &send]() { senderProcess(send); });
		}
	}
}

}

#endif
