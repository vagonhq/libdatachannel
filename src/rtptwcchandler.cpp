#if RTC_ENABLE_MEDIA

#include "rtptwcchandler.hpp"
#include "rtp.hpp"
#include <cassert>
namespace rtc {

TwccHandler::TwccHandler(uint8_t extId, std::function<void(message_vector &)> processPacketsCallback)
    : processPacketsCallback(processPacketsCallback) {
	assert(extId < 16);
	twccHeader.preparePacket(extId);
	twccSeqNum = 0;
}

void TwccHandler::outgoing(message_vector &messages, const message_callback &send) {
	message_vector outgoing;
	outgoing.reserve(messages.size());

	for (unsigned int i = 0; i < messages.size(); i++) {
		auto packet = messages[i];		
		auto rtpHeader = reinterpret_cast<RtpHeader *>(packet->data());
		outgoing.push_back(make_message(packet->size() + TWCC_EXT_HEADER_SIZE));
		uint8_t *src = reinterpret_cast<uint8_t *>(packet->data());
		uint8_t *dst = reinterpret_cast<uint8_t *>(outgoing.back()->data());
		memcpy(dst, src, rtpHeader->getSize());
		// set extension bit 1
		dst[0] = (dst[0] & ~0x10) | (0x01 << 4);
		src += rtpHeader->getSize();
		dst += rtpHeader->getSize();

		twccHeader.setTwccSeqNum(twccSeqNum++);
		memcpy(dst, &twccHeader, TWCC_EXT_HEADER_SIZE);
		dst += TWCC_EXT_HEADER_SIZE;
		memcpy(dst, src, packet->size() - rtpHeader->getSize());
	}

	if (processPacketsCallback)
		processPacketsCallback(outgoing);
	
	messages.swap(outgoing);
}


} // namespace rtc

#endif
