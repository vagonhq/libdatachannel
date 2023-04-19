#if RTC_ENABLE_MEDIA

#include "rtptwcchandler.hpp"

#include <cassert>
namespace rtc {

TwccHandler::TwccHandler(uint8_t extId, std::shared_ptr<ChainInterop> interop) {
	assert(extId < 16);
	twccHeader.preparePacket(extId);
	twccSeqNum = 0;
	twccInterop = interop;
}

ChainedOutgoingProduct TwccHandler::processOutgoingBinaryMessage(ChainedMessagesProduct messages,
                                                                 message_ptr control) {
	auto outgoing = make_chained_messages_product();
	outgoing->reserve(messages->size());
	uint16_t baseSeqNum = twccSeqNum;
	twccInterop->addFrame(baseSeqNum);

	for (unsigned int i = 0; i < messages->size(); i++) {
		auto packet = messages->at(i);
		auto rtpHeader = reinterpret_cast<RtpHeader *>(packet->data());
		outgoing->emplace_back(
		    std::move(std::make_shared<binary>(packet->size() + TWCC_EXT_HEADER_SIZE)));
		uint8_t *src = reinterpret_cast<uint8_t *>(packet->data());
		uint8_t *dst = reinterpret_cast<uint8_t *>(outgoing->back()->data());
		memcpy(dst, src, rtpHeader->getSize());
		// set extension bit 1
		dst[0] = (dst[0] & ~0x10) | (0x01 << 4);
		src += rtpHeader->getSize();
		dst += rtpHeader->getSize();

		twccHeader.setTwccSeqNum(twccSeqNum++);
		memcpy(dst, &twccHeader, TWCC_EXT_HEADER_SIZE);
		dst += TWCC_EXT_HEADER_SIZE;
		memcpy(dst, src, packet->size() - rtpHeader->getSize());

		twccInterop->addPacketToFrame(baseSeqNum, outgoing->back()->size());
	}

	return {outgoing, control};
}


} // namespace rtc

#endif
