#ifndef RTC_RTP_TWCC_HANDLER_H
#define RTC_RTP_TWCC_HANDLER_H

#if RTC_ENABLE_MEDIA

#include "mediahandler.hpp"
#include "chaininterop.hpp"
#include "rtp.hpp"
namespace rtc {

#define TWCC_EXT_HEADER_SIZE 8

class RTC_CPP_EXPORT TwccHandler final : public MediaHandler {
	RtpTwccExt twccHeader;
	uint16_t twccSeqNum;
	std::function<void(message_vector &)> processPacketsCallback;

public:
	TwccHandler(uint8_t extId, std::function<void(message_vector &)> processPacketsCallback);
	void outgoing(message_vector &messages, const message_callback &send) override;
};

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* RTC_RTP_TWCC_HANDLER_H */
