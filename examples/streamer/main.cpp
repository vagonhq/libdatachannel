/**
 * libdatachannel client example
 * Copyright (c) 2019-2020 Paul-Louis Ageneau
 * Copyright (c) 2019 Murat Dogan
 * Copyright (c) 2020 Will Munn
 * Copyright (c) 2020 Nico Chatzi
 * Copyright (c) 2020 Lara Mackey
 * Copyright (c) 2020 Erik Cota-Robles
 * Copyright (c) 2020 Filip Klembara (in2core)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "nlohmann/json.hpp"

#include "h264fileparser.hpp"
#include "opusfileparser.hpp"
#include "helpers.hpp"
#include "ArgParser.hpp"
#ifdef _WIN32
	#include <winsock.h>
#else
	#include <arpa/inet.h>
#endif
#include <limits>
#include <cmath>
#include <chrono>

using namespace rtc;
using namespace std;
using namespace std::chrono_literals;

using json = nlohmann::json;

template <class T> weak_ptr<T> make_weak_ptr(shared_ptr<T> ptr) { return ptr; }

/// all connected clients
unordered_map<string, shared_ptr<Client>> clients{};

/// Creates peer connection and client representation
/// @param config Configuration
/// @param wws Websocket for signaling
/// @param id Client ID
/// @returns Client
shared_ptr<Client> createPeerConnection(const Configuration &config,
                                        weak_ptr<WebSocket> wws,
                                        string id);

/// Creates stream
/// @param h264Samples Directory with H264 samples
/// @param fps Video FPS
/// @param opusSamples Directory with opus samples
/// @returns Stream object
shared_ptr<Stream> createStream(const string h264Samples, const unsigned fps, const string opusSamples);

/// Add client to stream
/// @param client Client
/// @param adding_video True if adding video
void addToStream(shared_ptr<Client> client, bool isAddingVideo);

/// Start stream
void startStream();

/// Main dispatch queue
DispatchQueue MainThread("Main");

/// Audio and video stream
optional<shared_ptr<Stream>> avStream = nullopt;

const string defaultRootDirectory = "../../../../../examples/streamer/samples/";
const string defaultH264SamplesDirectory = defaultRootDirectory + "h264/";
string h264SamplesDirectory = defaultH264SamplesDirectory;
const string defaultOpusSamplesDirectory = defaultRootDirectory + "opus/";
string opusSamplesDirectory = defaultOpusSamplesDirectory;
const string defaultIPAddress = "127.0.0.1";
const uint16_t defaultPort = 8000;
string ip_address = defaultIPAddress;
uint16_t port = defaultPort;

enum TwccPacketStatus {NOT_RECEIVED = 0, RECEIVED_SMALL_DELTA = 1, RECEIVED_LARGE_DELTA = 2, RESERVED = 3};
atomic_ulong receivedBps;
std::shared_ptr<ChainInterop> twccInterop = std::make_shared<ChainInterop>(1000);

struct TwccRunLengthChunk {
	uint16_t received;
	uint16_t not_received;
	TwccPacketStatus delta_info;
	TwccRunLengthChunk() {
		received = 0;
		not_received = 0;
		delta_info = TwccPacketStatus::NOT_RECEIVED;
	}
};

struct TwccStatusVectorChunk {
	uint8_t received;
	uint8_t not_received;
	std::vector<TwccPacketStatus> delta_info;
	std::vector<bool> is_received;

	TwccStatusVectorChunk() {
		received = 0;
		not_received = 0;
	}
};

struct TwccPacketInfo {
	uint16_t length;
	TwccPacketStatus delta_info;
	TwccPacketInfo(uint16_t length, TwccPacketStatus delta_info) : length(length), delta_info(delta_info) {}
};
template <typename T>
class RingBuffer {
	unsigned int buf_size;
	std::deque<T> list;

public:
	RingBuffer(unsigned int buf_size = 30) : buf_size(buf_size) { list = std::deque<T>(); }
	void insert(T val) {
		list.push_back(val);
		if (list.size() > buf_size)
			list.pop_front();
	}

	std::tuple<unsigned int, T> findMinDifference() const {
		T min = std::numeric_limits<T>::infinity();
		T temp;
		unsigned int min_idx;
		for (unsigned int i = 1; i < list.size(); i++) {
			temp = list.at(i) - list.at(i - 1);
			if (temp < min) {
				min = temp;
				min_idx = i;
			}
		}

		return std::make_tuple(min_idx, min);
	}

	size_t size() const { return list.size(); }
	T back() const { return list.back(); }
	T at(unsigned int i) const { return list.at(i); }
	typename std::deque<T>::iterator begin() { return list.begin(); }
	typename std::deque<T>::iterator end() { return list.end(); }
};


enum class OverUseDetectorState {UNDERUSE, NORMAL, OVERUSE};
enum class RateControlState {DECREASE, HOLD, INCREASE};
enum class ConvergenceState {CLOSE, DISTANT};

const char* detectorStateToString(OverUseDetectorState state) {
	switch (state) {
	case OverUseDetectorState::UNDERUSE:
		return "Underuse";
		break;
	case OverUseDetectorState::NORMAL:
		return "Normal";
		break;
	case OverUseDetectorState::OVERUSE:
		return "Overuse";
		break;
	default:
		return "Undefined detector state";
		break;
	}
}

const char *rateControlStateToString(RateControlState state) {
	switch (state) {
	case RateControlState::DECREASE:
		return "Decrease";
		break;
	case RateControlState::HOLD:
		return "Hold";
		break;
	case RateControlState::INCREASE:
		return "Increase";
		break;
	default:
		return "Undefined RC state";
		break;
	}
}

const char *convergenceStateStateToString(ConvergenceState state) {
	switch (state) {
	case ConvergenceState::CLOSE:
		return "Close";
		break;
	case ConvergenceState::DISTANT:
		return "Distant";
		break;
	default:
		return "Undefined convergence state";
		break;
	}
}

class StatsCalculator {
	const double alpha = 0.95;
	unsigned long long count;
	double mean, m_2, old_mean, std, old_std;

public:
	StatsCalculator() : count(0), mean(0), m_2(0), old_mean(0), std(0), old_std(0) {}
	
	void add(double x) {
		count++;
		old_mean = mean;
		old_std = std;

		double delta = x - mean;
		mean += delta / count;
		double delta2 = x - mean;
		m_2 += delta * delta2;
		if (count > 2) {
			std = sqrt(m_2 / (count - 1));
			std = std * alpha + old_std * (1 - alpha);
			mean = mean * alpha + old_mean * (1 - alpha);
		}
	}

	std::tuple<double, double> getMeanStd() {
		if (count > 2) {
			return std::make_tuple(mean, std);
		}
		return std::make_tuple(0, 0);
	}

	bool isValid() { return count > 2 ? true : false; }

	void reset() {
		count = 0;
		mean = 0;
		m_2 = 0;
		old_mean = 0;
		std = 0;
		old_std = 0;
	}
};

class CCResponder final : public rtc::MediaHandler {
	uint32_t m_ssrc;

	// in bits per second
	int m_bandwidth = 0;
	int m_max_bandwidth = 0;
	float m_estimate = 0.0;

	int m_last_processed_seqnum = 0;
	int m_last_seen_seqnum = 0;
	time_t m_last_processed_at = 0;

	// weighted total - will be used to calculated weighted average
	float m_current_loss_total = 0.0;

	std::function<void(int)> m_change_bandwidth;
	
	// delay based rate control
	// kalman filter
	const float q = 0.001, chi = 0.01;
	float m_i = 0, m_i_1 = 0, var_v = 1, e_i = 0.1, alpha = 0.5;
	// over-use detector
	const float del_var_th_max = 600, del_var_th_min = 6;
	const unsigned int overuse_time_th_ms = 10;
	const float detector_k_u = 0.01, detector_k_d = 0.00018;
	float del_var_th = 12.5;
	std::chrono::steady_clock::time_point previous_detector_time;
	OverUseDetectorState prev_detector_state = OverUseDetectorState::NORMAL;

	uint32_t inter_arrival_est = 0;
	uint16_t last_twcc_packet_seqnum = 0;
	RingBuffer<double> arrival_time_ms = RingBuffer<double>();
	RingBuffer<uint32_t> arrival_timestamps = RingBuffer<uint32_t>();
	RingBuffer<uint16_t> seqnums = RingBuffer<uint16_t>();
	RingBuffer<double> deltas = RingBuffer<double>();
	uint32_t reference_timestamp = 0;

	// rate control
	std::function<std::optional<std::chrono::milliseconds>()> rtt_func;
	RateControlState rate_control_state = RateControlState::INCREASE;
	ConvergenceState convergence_state = ConvergenceState::DISTANT;

	std::chrono::steady_clock::time_point previous_rc_time;
	std::chrono::steady_clock::time_point previous_delay_bwe_time;
	const float beta = 0.85;
	float r_hat, a_hat;
	unsigned int fps = 30; // TODO: move this to constructor

	StatsCalculator bw_stats = StatsCalculator();
	std::shared_ptr<ChainInterop> twccInterop;

	void update_delay_filter(float d_i) {
		float z_i, k_i, P;
		// prediction step m_i = m_i_1 and P = e_i + q
		m_i_1 = m_i;
		P = e_i + q;
		z_i = d_i - m_i; // measurement error
		// update
		k_i = (P) / (var_v + P);
		if (z_i > 3 * sqrt(var_v))
			m_i = m_i_1 + k_i * 3 * sqrt(var_v);
		else
			m_i = m_i_1 + k_i * z_i;

		e_i = (1 - k_i) * P;

		var_v = alpha * var_v + (1 - alpha) * z_i * z_i;
		var_v = var_v < 1 ? 1 : var_v;
		alpha = pow(1 - chi, fps / (5000.0)); // This formula is from Interceptor API of Pion

		//std::cout << "mi " << m_i << " mi_1 " << m_i_1 << " var_v " << var_v << " e_i " << e_i
		//          << " alpha " << alpha << " fmax " << f_max << " size " << arrival_time_ms.size()
		//          << std::endl;
	}

	OverUseDetectorState run_overuse_detector() {
		OverUseDetectorState result;
		if (m_i > del_var_th)
			result = OverUseDetectorState::OVERUSE;
		else if (m_i < -del_var_th)
			result = OverUseDetectorState::UNDERUSE;
		else
			result = OverUseDetectorState::NORMAL;
		/*std::cout << "overuse prev " << detectorStateToString(prev_detector_state) << " new "
		          << detectorStateToString(result) << std::endl;*/
		auto time_now = std::chrono::steady_clock::now();
		if (result != prev_detector_state) {
			previous_detector_time = time_now;
		} else {
			if (result == OverUseDetectorState::OVERUSE) {
				auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
				                        time_now - previous_detector_time)
				                        .count();
				if (m_i < m_i_1 || milliseconds < overuse_time_th_ms)
					result = OverUseDetectorState::NORMAL;
			}
			prev_detector_state = result;
		}

		if (abs(m_i) <= del_var_th + 15) {
			float inter_arrival = twccInterop->findArrivalIntervalLastTwoFramesMS();
			float k = abs(m_i) < del_var_th ? detector_k_d : detector_k_u;

			float temp = del_var_th + inter_arrival * k * (abs(m_i) - del_var_th);
			if (temp > del_var_th_max)
				temp = del_var_th_max;
			else if (temp < del_var_th_min)
				temp = del_var_th_min;
			del_var_th = temp;
			//std::cout << "del_var_th update " << temp << " interarrival " << inter_arrival << std::endl;
		}

		return result;
	}

	void run_rate_control() {
		auto time_now = std::chrono::steady_clock::now();
		auto time_since_last_update_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - previous_rc_time).count();
		previous_rc_time = time_now;
		r_hat = receivedBps.load() * 8;

		auto bpsServer = twccInterop->getReceivedBitsPerSecond();
		if (rate_control_state == RateControlState::DECREASE)
			bw_stats.add(r_hat);
		
		if (rate_control_state == RateControlState::INCREASE) {
			if (bw_stats.isValid()) {
				auto [mean_r, std_r] = bw_stats.getMeanStd();
				auto three_sigma = 3 * std_r;
				if (r_hat < mean_r + three_sigma || r_hat > mean_r - three_sigma)
					convergence_state = ConvergenceState::CLOSE;
				else {
					convergence_state = ConvergenceState::DISTANT;
					bw_stats.reset();
				}
			} else
				convergence_state = ConvergenceState::DISTANT;

			if (convergence_state == ConvergenceState::DISTANT) {
				float eta = pow(1.08, fmin(time_since_last_update_ms / 1000.0, 1.0));
				a_hat = eta * a_hat;
			} else {
				float rtt = 0;
				if (auto rtt_opt = rtt_func()) {
					rtt = std::chrono::duration_cast<std::chrono::milliseconds>(rtt_opt.value()).count();
				}
				float response_time_ms = 100 + rtt;
				alpha = 0.5 * fmin((float)time_since_last_update_ms / response_time_ms, 1.0);
				float bits_per_frame = a_hat / fps;
				float packets_per_frame = ceil(bits_per_frame / (1200 * 8));
				float avg_packet_size_bits = bits_per_frame / packets_per_frame;
				a_hat = a_hat + fmax(1000, alpha * avg_packet_size_bits);
			}
			if (a_hat > r_hat * 1.5)
				a_hat = r_hat * 1.5;
		} else if (rate_control_state == RateControlState::DECREASE) {
			a_hat = beta * r_hat;
		}

		std::cout << "ahat " << a_hat << " rhat " << r_hat << " bpsserver " << bpsServer << " "
		          << twccInterop->size()
		          << std::endl;
	}

	void process_fraction(uint32_t ssrc, uint8_t frac_byte, int highest_seqnum) {

		if (m_bandwidth == 0) {
			int initial_bandwidth = 100000;
			m_bandwidth = initial_bandwidth;
			m_max_bandwidth = initial_bandwidth;
			m_estimate = initial_bandwidth;
			// Logf("initial bandwidth %d", initial_bandwidth);
		}

		if (m_last_seen_seqnum == 0) {
			// we need to discard the first reader report for the weighted averaging to work
			// they are frequent enough that discarding the first report shouldn't be much of a
			// problem
			m_last_seen_seqnum = highest_seqnum;
			m_last_processed_seqnum = highest_seqnum;
			m_last_processed_at = time(NULL);
			return;
		}

		// no need to processs duplicates
		if (highest_seqnum <= m_last_seen_seqnum)
			return;

		// this basically never happens but we shouldn't process these messages anyway
		if (ssrc != m_ssrc)
			return;

		float frac = frac_byte;
		frac /= 256.0;

		int current_frac_delta = highest_seqnum - m_last_seen_seqnum;
		m_current_loss_total += current_frac_delta * frac;

		m_last_seen_seqnum = highest_seqnum;

		time_t now = time(NULL);

		if (now - m_last_processed_at > 1) {
			int total_frac_delta = m_last_seen_seqnum - m_last_processed_seqnum;

			update_estimate(m_current_loss_total / total_frac_delta);
			m_last_processed_seqnum = highest_seqnum;
			m_current_loss_total = 0.0;
			m_last_processed_at = now;
		}
	}

	void update_estimate(float loss_fraction) {
		// this implements the algorithm described in
		// https://datatracker.ietf.org/doc/html/draft-ietf-rmcat-gcc-02#section-6 Not the best
		// since it relies on packet loss only

		bool estimate_changed = false;

		// Logf("update_estimate: old estimate % f fraction % f", m_estimate, loss_fraction);

		if (loss_fraction < 0.015f) {
			m_estimate *= 1.05;
			estimate_changed = true;
		}
		if (loss_fraction > 0.05f) {
			m_estimate *= (1 - 0.5 * loss_fraction);
			estimate_changed = true;
		}

		if (m_estimate > m_max_bandwidth)
			m_estimate = m_max_bandwidth;

		if (estimate_changed) {
			// Logf("estimate now %f", m_estimate);
			int int_estimate = m_estimate;

			if (abs(int_estimate - m_bandwidth) > 1000000) {
				int_estimate = 1000000 * round(m_estimate / 1000000.f);

				if (int_estimate > m_max_bandwidth)
					int_estimate = m_max_bandwidth;
				if (int_estimate < 2000000)
					int_estimate = 2000000;

				if (int_estimate != m_bandwidth) {
					m_bandwidth = int_estimate;
					// Logf("Change bandwidth to %d", m_bandwidth);

					m_change_bandwidth(m_bandwidth);
				}
			}
		}
	}

	bool isStatusVector(uint16_t data) { return (data >> 15) & 0x01; }

	uint8_t getDeltaInfo(uint16_t data) { return (data >> 13) & 0x03; }

	uint16_t getRunLength(uint16_t data) { return data & 0x1FFF; }
	
	TwccRunLengthChunk processRunLength(uint16_t data) {
		TwccRunLengthChunk result = TwccRunLengthChunk();
		result.delta_info = static_cast<TwccPacketStatus>(getDeltaInfo(data));
		if (result.delta_info)
			result.received = getRunLength(data);
		else
			result.not_received = getRunLength(data);

		return result;
	}

	TwccStatusVectorChunk processStatusVector(uint16_t data, uint16_t packets_remaining) {
		TwccStatusVectorChunk result = TwccStatusVectorChunk();
		bool symbol_size = (data >> 14) & 0x01;
		uint8_t received = 0;
		// 2 bits for each packet
		if (symbol_size) {
			const uint8_t select = 0x03;
			uint8_t status;
			uint16_t n = packets_remaining < 7 ? packets_remaining : 7;
			for (unsigned int i = 0; i < n; i++) {
				status = (uint8_t)((data >> (2 * (6 - i))) & select);
				result.delta_info.push_back(static_cast<TwccPacketStatus>(status));
				if (status == 0) {
					result.not_received += 1;
					result.is_received.push_back(false);
				} else {
					result.received += 1;
					result.is_received.push_back(true);
				}
			}
		} else {
			const uint8_t select = 0x01;
			uint8_t status;
			uint16_t n = packets_remaining < 14 ? packets_remaining : 14;
			for (unsigned int i = 0; i < n; i++) {
				status = (uint8_t)((data >> (13 - i)) & select);
				result.delta_info.push_back(static_cast<TwccPacketStatus>(status));
				if (status == 0) {
					result.not_received += 1;
					result.is_received.push_back(false);
				}
				else {
					result.received += 1;
					result.is_received.push_back(true);
				}
			}
		}

		return result;
	}

	void state_transition(OverUseDetectorState detector_state) {
		switch (detector_state) {
		case OverUseDetectorState::UNDERUSE:
			if (rate_control_state == RateControlState::INCREASE ||
			    rate_control_state == RateControlState::DECREASE)
				rate_control_state = RateControlState::HOLD;
			break;
		case OverUseDetectorState::NORMAL:
			if (rate_control_state == RateControlState::HOLD)
				rate_control_state = RateControlState::INCREASE;
			else if (rate_control_state == RateControlState::DECREASE)
				rate_control_state = RateControlState::HOLD;
			break;
		case OverUseDetectorState::OVERUSE:
			if (rate_control_state == RateControlState::HOLD ||
			    rate_control_state == RateControlState::INCREASE)
				rate_control_state = RateControlState::DECREASE;
			break;
		default:
			break;
		}
	}

public:
	CCResponder(uint32_t ssrc, std::function<void(int)> callback,
	            std::function<std::optional<std::chrono::milliseconds>()> rtt_callback,
	            int initial_bw_bps, std::shared_ptr<ChainInterop> interop)
	    : m_ssrc(ssrc), m_change_bandwidth(callback), rtt_func(rtt_callback), r_hat(initial_bw_bps),
	      a_hat(initial_bw_bps), twccInterop(interop) {
		previous_detector_time = std::chrono::steady_clock::now();
	}

	rtc::message_ptr incoming(rtc::message_ptr message) override {
		// Can't parse header in that case
		// std::cout << "f " << message->size() << std::endl;
		if (message->size() < 4)
			return message;

		uint8_t *byte_msg = (uint8_t *)message->data();
		uint16_t *short_msg = (uint16_t *)message->data();

		uint8_t packet_type = byte_msg[1];

		// Logf("message size %d packet type %d", message->size(), packet_type);
		//std::cout << "cc packet type" << (int)packet_type << std::endl;
		// checking for receiver reports
		if (packet_type == 201) {
			auto time_now = std::chrono::steady_clock::now();
			// 8 byte header + 24 byte report blocks
			uint16_t length = message->size();
			size_t num_blocks = (length - 8) / 24;
			// Logf("length %d num_blocks %d", length, num_blocks);

			uint8_t *blocks_start = byte_msg + 8;
			for (size_t i = 0; i < num_blocks; i++) {
				uint8_t *byte_block = (uint8_t *)(blocks_start + 24 * i);
				uint32_t *long_block = (uint32_t *)(blocks_start + 24 * i);

				uint32_t ssrc = htonl(long_block[0]);
				uint32_t seqnum = htonl(long_block[2]);
				uint8_t byte_frac = byte_block[4];

				process_fraction(ssrc, byte_frac, seqnum);

				// Logf("receiver report ssrc %d seqnum %d packets lost pct %.2f", ssrc, seqnum,
				// frac);
			}

			// jitter
			auto rtcp_header = reinterpret_cast<RtcpRr *>(message->data());
			uint32_t new_jitter = 0;
			for (size_t i = 0; i < num_blocks; i++) {
				auto rtcp_report = rtcp_header->getReportBlock(i);
				if (m_ssrc == rtcp_report->getSSRC())
					new_jitter = rtcp_report->jitter();
			}
			auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - previous_delay_bwe_time).count();
			if (arrival_time_ms.size() > 10 && elapsed_ms > 500) {
				previous_delay_bwe_time = time_now;
				update_delay_filter(new_jitter);
				
				auto detector_state = run_overuse_detector();
				state_transition(detector_state);
				std::cout << "states " << detectorStateToString(detector_state) << " "
				          << rateControlStateToString(rate_control_state) << " "
				          << convergenceStateStateToString(convergence_state) << std::endl;
				run_rate_control();
			}
		} else if (packet_type == 205) {
			auto rtcp = reinterpret_cast<RtcpTwcc *>(message->data());
			// FIXME: handle wraparound
			auto newSeqNum = rtcp->getBaseSeqNum();
			if (last_twcc_packet_seqnum < newSeqNum) {
				last_twcc_packet_seqnum = newSeqNum;
			} else {
				if (newSeqNum < 1000 && last_twcc_packet_seqnum > 65000)
					last_twcc_packet_seqnum = newSeqNum;
				else
					return message;
			}


			auto len_in_bytes = rtcp->header.header.lengthInBytes() - 20;
			auto num_packets = rtcp->getPacketStatusCount();
			std::vector<TwccPacketInfo> packet_info;
			std::vector<bool> isReceived;
			std::vector<double> packet_arrivals;
			uint32_t arrival_ts = rtcp->getReferenceTime() * 64;

			auto root = rtcp->getBody();
			auto p_body = rtcp->getBody();
			unsigned int byte_counter = 0;
			unsigned int num_received_packets = 0;
			unsigned int num_not_received_packets = 0;
			unsigned int packets_counted = 0;
			double delta_sum = 0;
			
			if (arrival_time_ms.size() == 0) {
				reference_timestamp = arrival_ts;
				arrival_ts = 0;
			} else
				arrival_ts -= reference_timestamp;

			while (byte_counter < len_in_bytes && packets_counted < num_packets) {
				auto packet_chunk = ntohs(* reinterpret_cast<uint16_t *>(p_body));
				if (isStatusVector(packet_chunk)) {
					TwccStatusVectorChunk packet_result = processStatusVector(packet_chunk, num_packets - packets_counted);
					num_received_packets += packet_result.received;
					num_not_received_packets += packet_result.not_received;
					isReceived.insert(isReceived.end(), packet_result.is_received.begin(), packet_result.is_received.end());
					for (unsigned int i = 0; i < packet_result.received + packet_result.not_received; i++) {
						packet_info.emplace_back(1, packet_result.delta_info[i]);
					}
				} else {
					TwccRunLengthChunk packet_result = processRunLength(packet_chunk);
					if (packet_result.received) {
						num_received_packets += packet_result.received;
						for (int i = 0; i < packet_result.received; i++)
							isReceived.push_back(true);
					}
					if (packet_result.not_received) {
						num_not_received_packets += packet_result.not_received;
						for (int i = 0; i < packet_result.not_received; i++)
							isReceived.push_back(false);
					}
					
					switch (packet_result.delta_info) {
					case TwccPacketStatus::NOT_RECEIVED:
						packet_info.emplace_back(packet_result.not_received,
						                         TwccPacketStatus::NOT_RECEIVED);
						break;
					case TwccPacketStatus::RECEIVED_SMALL_DELTA:
						packet_info.emplace_back(packet_result.received,
						                         TwccPacketStatus::RECEIVED_SMALL_DELTA);
						break;
					case TwccPacketStatus::RECEIVED_LARGE_DELTA:
						packet_info.emplace_back(packet_result.received,
						                         TwccPacketStatus::RECEIVED_LARGE_DELTA);
						break;
					default:
						break;
					}
				}
				byte_counter += 2;
				p_body += 2;
				packets_counted = num_received_packets + num_not_received_packets;
			}

			for (unsigned int i = 0; i < packet_info.size(); i++) {
				auto info = packet_info[i];
				for (unsigned int j = 0; j < info.length; j++) {
					double temp;
					switch (info.delta_info) { 
					case TwccPacketStatus::RECEIVED_SMALL_DELTA:
						delta_sum += (double)(*reinterpret_cast<uint8_t *>(p_body)) * 0.25;
						p_body += 1;
						byte_counter += 1;
						packet_arrivals.push_back(arrival_ts + delta_sum);
						break;
					case TwccPacketStatus::RECEIVED_LARGE_DELTA:
						temp = (double)ntohs(*reinterpret_cast<int16_t *>(p_body)) * 0.25;
						if (temp < 0)
							std::cout << "negative delta" << std::endl;
						delta_sum += temp;
						p_body += 2;
						byte_counter += 2;
						packet_arrivals.push_back(arrival_ts + delta_sum);
						break;
					default:
						packet_arrivals.push_back(0);
						break;
					}
				}
			}

			twccInterop->updateReceivedStatus(newSeqNum, isReceived, packet_arrivals);
			arrival_timestamps.insert(arrival_ts);
			seqnums.insert(newSeqNum);
			arrival_time_ms.insert(arrival_ts + delta_sum);
			deltas.insert(delta_sum);
			//std::cout << "arrival_ts " << (int)arrival_ts << " deltas " << (int)delta_sum
			//          << " arrival_time_ms " << (int) (arrival_ts + delta_sum) << " seqnum "
			//          << newSeqNum << std::endl;
			if (byte_counter < len_in_bytes) {
				if (len_in_bytes - byte_counter >= 4)
					std::cout << "TWCC packet size-delta mismatch" << std::endl;
			}
		}

		return message;
	}

	rtc::message_ptr outgoing(rtc::message_ptr message) override { return message; }
};

/// Incomming message handler for websocket
/// @param message Incommint message
/// @param config Configuration
/// @param ws Websocket
void wsOnMessage(json message, Configuration config, shared_ptr<WebSocket> ws) {
    auto it = message.find("id");
    if (it == message.end())
        return;
    string id = it->get<string>();

    it = message.find("type");
    if (it == message.end())
        return;
    string type = it->get<string>();

    if (type == "request") {
        clients.emplace(id, createPeerConnection(config, make_weak_ptr(ws), id));
    } else if (type == "answer") {
        if (auto jt = clients.find(id); jt != clients.end()) {
            auto pc = jt->second->peerConnection;
            auto sdp = message["sdp"].get<string>();
            auto description = Description(sdp, type);
            pc->setRemoteDescription(description);
        }
    }
}

int main(int argc, char **argv) try {
    bool enableDebugLogs = false;
    bool printHelp = false;
    int c = 0;
    auto parser = ArgParser({{"a", "audio"}, {"b", "video"}, {"d", "ip"}, {"p","port"}}, {{"h", "help"}, {"v", "verbose"}});
    auto parsingResult = parser.parse(argc, argv, [](string key, string value) {
        if (key == "audio") {
            opusSamplesDirectory = value + "/";
        } else if (key == "video") {
            h264SamplesDirectory = value + "/";
        } else if (key == "ip") {
            ip_address = value;
        } else if (key == "port") {
            port = atoi(value.data());
        } else {
            cerr << "Invalid option --" << key << " with value " << value << endl;
            return false;
        }
        return true;
    }, [&enableDebugLogs, &printHelp](string flag){
        if (flag == "verbose") {
            enableDebugLogs = true;
        } else if (flag == "help") {
            printHelp = true;
        } else {
            cerr << "Invalid flag --" << flag << endl;
            return false;
        }
        return true;
    });
    if (!parsingResult) {
        return 1;
    }

    if (printHelp) {
        cout << "usage: stream-h264 [-a opus_samples_folder] [-b h264_samples_folder] [-d ip_address] [-p port] [-v] [-h]" << endl
        << "Arguments:" << endl
        << "\t -a " << "Directory with opus samples (default: " << defaultOpusSamplesDirectory << ")." << endl
        << "\t -b " << "Directory with H264 samples (default: " << defaultH264SamplesDirectory << ")." << endl
        << "\t -d " << "Signaling server IP address (default: " << defaultIPAddress << ")." << endl
        << "\t -p " << "Signaling server port (default: " << defaultPort << ")." << endl
        << "\t -v " << "Enable debug logs." << endl
        << "\t -h " << "Print this help and exit." << endl;
        return 0;
    }
    if (enableDebugLogs) {
        InitLogger(LogLevel::Debug);
    }

    Configuration config;
    string stunServer = "stun:stun.l.google.com:19302";
    cout << "STUN server is " << stunServer << endl;
    config.iceServers.emplace_back(stunServer);
    config.disableAutoNegotiation = true;

    string localId = "server";
    cout << "The local ID is: " << localId << endl;

    auto ws = make_shared<WebSocket>();

    ws->onOpen([]() { cout << "WebSocket connected, signaling ready" << endl; });

    ws->onClosed([]() { cout << "WebSocket closed" << endl; });

    ws->onError([](const string &error) { cout << "WebSocket failed: " << error << endl; });

    ws->onMessage([&](variant<binary, string> data) {
        if (!holds_alternative<string>(data))
            return;

        json message = json::parse(get<string>(data));
        MainThread.dispatch([message, config, ws]() {
            wsOnMessage(message, config, ws);
        });
    });

    const string url = "ws://" + ip_address + ":" + to_string(port) + "/" + localId;
    cout << "URL is " << url << endl;
    ws->open(url);

    cout << "Waiting for signaling to be connected..." << endl;
    while (!ws->isOpen()) {
        if (ws->isClosed())
            return 1;
        this_thread::sleep_for(100ms);
    }

    while (true) {
        string id;
        cout << "Enter to exit" << endl;
        cin >> id;
        cin.ignore();
        cout << "exiting" << endl;
        break;
    }

    cout << "Cleaning up..." << endl;
    return 0;

} catch (const std::exception &e) {
    std::cout << "Error: " << e.what() << std::endl;
    return -1;
}

shared_ptr<ClientTrackData> addVideo(const shared_ptr<PeerConnection> pc, const uint8_t payloadType, const uint32_t ssrc, const string cname, const string msid, const function<void (void)> onOpen) {
    auto video = Description::Video(cname);
    video.addH264Codec(payloadType);
    video.addSSRC(ssrc, cname, msid, cname);
	video.addExtMap(rtc::Description::Entry::ExtMap("3 http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01"));
    auto track = pc->addTrack(video);
    // create RTP configuration
    auto rtpConfig = make_shared<RtpPacketizationConfig>(ssrc, cname, payloadType, H264RtpPacketizer::defaultClockRate);
    // create packetizer
    auto packetizer = make_shared<H264RtpPacketizer>(NalUnit::Separator::Length, rtpConfig);
    // add RTCP SR handler
    auto srReporter = make_shared<RtcpSrReporter>(rtpConfig);
    packetizer->addToChain(srReporter);
    // add RTCP NACK handler
    auto nackResponder = make_shared<RtcpNackResponder>();
    packetizer->addToChain(nackResponder);
    // set handler
    track->setMediaHandler(packetizer);
    track->onOpen(onOpen);
    auto trackData = make_shared<ClientTrackData>(track, srReporter);
    return trackData;
}

shared_ptr<ClientTrackData> addAudio(const shared_ptr<PeerConnection> pc, const uint8_t payloadType, const uint32_t ssrc, const string cname, const string msid, const function<void (void)> onOpen) {
    auto audio = Description::Audio(cname);
    audio.addOpusCodec(payloadType);
    audio.addSSRC(ssrc, cname, msid, cname);
    auto track = pc->addTrack(audio);
    // create RTP configuration
    auto rtpConfig = make_shared<RtpPacketizationConfig>(ssrc, cname, payloadType, OpusRtpPacketizer::DefaultClockRate);
    // create packetizer
    auto packetizer = make_shared<OpusRtpPacketizer>(rtpConfig);
    // add RTCP SR handler
    auto srReporter = make_shared<RtcpSrReporter>(rtpConfig);
    packetizer->addToChain(srReporter);
    // add RTCP NACK handler
    auto nackResponder = make_shared<RtcpNackResponder>();
    packetizer->addToChain(nackResponder);
    // set handler
    track->setMediaHandler(packetizer);
    track->onOpen(onOpen);
    auto trackData = make_shared<ClientTrackData>(track, srReporter);
    return trackData;
}

// Create and setup a PeerConnection
shared_ptr<Client> createPeerConnection(const Configuration &config,
                                                weak_ptr<WebSocket> wws,
                                                string id) {
	receivedBps.store(0);
	auto pc = make_shared<PeerConnection>(config);
    auto client = make_shared<Client>(pc);
	pc->setMediaHandler(std::make_shared<CCResponder>(
	    1, [&](int bandwidth) { std::cout << "bw chnge " << bandwidth << std::endl; },
	    [wpc = make_weak_ptr(pc)]() -> std::optional<std::chrono::milliseconds>{
		    if (auto pc = wpc.lock())
			    return pc->rtt();
		    else
			    std::cout << "pc pointer is empty!" << std::endl;
		    return std::nullopt;
	    }, 2000000, twccInterop));
	pc->onStateChange([id](PeerConnection::State state) {
        cout << "State: " << state << endl;
        if (state == PeerConnection::State::Disconnected ||
            state == PeerConnection::State::Failed ||
            state == PeerConnection::State::Closed) {
            // remove disconnected client
            MainThread.dispatch([id]() {
                clients.erase(id);
            });
        }
    });

    pc->onGatheringStateChange(
        [wpc = make_weak_ptr(pc), id, wws](PeerConnection::GatheringState state) {
        cout << "Gathering State: " << state << endl;
        if (state == PeerConnection::GatheringState::Complete) {
            if(auto pc = wpc.lock()) {
                auto description = pc->localDescription();
                json message = {
                    {"id", id},
                    {"type", description->typeString()},
                    {"sdp", string(description.value())}
                };
                // Gathering complete, send answer
                if (auto ws = wws.lock()) {
                    ws->send(message.dump());
                }
            }
        }
    });

    client->video = addVideo(pc, 102, 1, "video-stream", "stream1", [id, wc = make_weak_ptr(client)]() {
        MainThread.dispatch([wc]() {
            if (auto c = wc.lock()) {
                addToStream(c, true);
            }
        });
        cout << "Video from " << id << " opened" << endl;
    });

    client->audio = addAudio(pc, 111, 2, "audio-stream", "stream1", [id, wc = make_weak_ptr(client)]() {
        MainThread.dispatch([wc]() {
            if (auto c = wc.lock()) {
                addToStream(c, false);
            }
        });
        cout << "Audio from " << id << " opened" << endl;
    });

    auto dc = pc->createDataChannel("ping-pong");
	auto dcBps = pc->createDataChannel("Bytes");

	dcBps->onOpen([id, wdc = make_weak_ptr(dcBps)]() {
		if (auto dc = wdc.lock()) {
			dc->send("Second dc");
		}
	});

	dcBps->onMessage(nullptr, [id, wdc = make_weak_ptr(dcBps)](string msg) {
		// cout << "Message from " << id << " received: " << msg << endl;
		std::string::size_type idx = msg.find("Bps:");
		if (idx != std::string::npos) {
			unsigned int received = 0;
			try {
				received = stoul(msg.substr(idx + 4));
				receivedBps.store(received);
				//std::cout << "received Bps: " << received << std::endl;
			} catch (std::invalid_argument const &ex) {
				std::cout << "Invalid Arg " << msg << std::endl;
			} catch (const std::out_of_range &e) {
				std::cout << "Bps out of range " << msg << std::endl;
			}
		}
	});
	
    dc->onOpen([id, wdc = make_weak_ptr(dc)]() {
        if (auto dc = wdc.lock()) {
            dc->send("Ping");
        }
    });

    dc->onMessage(nullptr, [id, wdc = make_weak_ptr(dc)](string msg) {
        //cout << "Message from " << id << " received: " << msg << endl;
        if (auto dc = wdc.lock()) {
            dc->send("Ping");
        }
    });
    client->dataChannel = dc;
	client->dataChannel2 = dcBps;
    pc->setLocalDescription();
    return client;
};

/// Create stream
shared_ptr<Stream> createStream(const string h264Samples, const unsigned fps, const string opusSamples) {
    // video source
    auto video = make_shared<H264FileParser>(h264Samples, fps, true);
    // audio source
    auto audio = make_shared<OPUSFileParser>(opusSamples, true);

    auto stream = make_shared<Stream>(video, audio);
    // set callback responsible for sample sending
    stream->onSample([ws = make_weak_ptr(stream)](Stream::StreamSourceType type, uint64_t sampleTime, rtc::binary sample) {
        vector<ClientTrack> tracks{};
        string streamType = type == Stream::StreamSourceType::Video ? "video" : "audio";
        // get track for given type
        function<optional<shared_ptr<ClientTrackData>> (shared_ptr<Client>)> getTrackData = [type](shared_ptr<Client> client) {
            return type == Stream::StreamSourceType::Video ? client->video : client->audio;
        };
        // get all clients with Ready state
        for(auto id_client: clients) {
            auto id = id_client.first;
            auto client = id_client.second;
            auto optTrackData = getTrackData(client);
            if (client->getState() == Client::State::Ready && optTrackData.has_value()) {
                auto trackData = optTrackData.value();
                tracks.push_back(ClientTrack(id, trackData));
            }
        }
        if (!tracks.empty()) {
            for (auto clientTrack: tracks) {
                auto client = clientTrack.id;
                auto trackData = clientTrack.trackData;
                auto rtpConfig = trackData->sender->rtpConfig;

                // sample time is in us, we need to convert it to seconds
                auto elapsedSeconds = double(sampleTime) / (1000 * 1000);
                // get elapsed time in clock rate
                uint32_t elapsedTimestamp = rtpConfig->secondsToTimestamp(elapsedSeconds);
                // set new timestamp
                rtpConfig->timestamp = rtpConfig->startTimestamp + elapsedTimestamp;

                // get elapsed time in clock rate from last RTCP sender report
                auto reportElapsedTimestamp = rtpConfig->timestamp - trackData->sender->lastReportedTimestamp();
                // check if last report was at least 1 second ago
                if (rtpConfig->timestampToSeconds(reportElapsedTimestamp) > 1) {
                    trackData->sender->setNeedsToReport();
                }

                //cout << "Sending " << streamType << " sample with size: " << to_string(sample.size()) << " to " << client << endl;
                try {
                    // send sample
                    trackData->track->send(sample);
                } catch (const std::exception &e) {
                    cerr << "Unable to send "<< streamType << " packet: " << e.what() << endl;
                }
            }
        }
        MainThread.dispatch([ws]() {
            if (clients.empty()) {
                // we have no clients, stop the stream
                if (auto stream = ws.lock()) {
                    stream->stop();
                }
            }
        });
    });
    return stream;
}

/// Start stream
void startStream() {
    shared_ptr<Stream> stream;
    if (avStream.has_value()) {
        stream = avStream.value();
        if (stream->isRunning) {
            // stream is already running
            return;
        }
    } else {
        stream = createStream(h264SamplesDirectory, 30, opusSamplesDirectory);
        avStream = stream;
    }
    stream->start();
}

/// Send previous key frame so browser can show something to user
/// @param stream Stream
/// @param video Video track data
void sendInitialNalus(shared_ptr<Stream> stream, shared_ptr<ClientTrackData> video) {
    auto h264 = dynamic_cast<H264FileParser *>(stream->video.get());
    auto initialNalus = h264->initialNALUS();

    // send previous NALU key frame so users don't have to wait to see stream works
    if (!initialNalus.empty()) {
        const double frameDuration_s = double(h264->getSampleDuration_us()) / (1000 * 1000);
        const uint32_t frameTimestampDuration = video->sender->rtpConfig->secondsToTimestamp(frameDuration_s);
        video->sender->rtpConfig->timestamp = video->sender->rtpConfig->startTimestamp - frameTimestampDuration * 2;
        video->track->send(initialNalus);
        video->sender->rtpConfig->timestamp += frameTimestampDuration;
        // Send initial NAL units again to start stream in firefox browser
        video->track->send(initialNalus);
    }
}

/// Add client to stream
/// @param client Client
/// @param adding_video True if adding video
void addToStream(shared_ptr<Client> client, bool isAddingVideo) {
    if (client->getState() == Client::State::Waiting) {
        client->setState(isAddingVideo ? Client::State::WaitingForAudio : Client::State::WaitingForVideo);
    } else if ((client->getState() == Client::State::WaitingForAudio && !isAddingVideo)
               || (client->getState() == Client::State::WaitingForVideo && isAddingVideo)) {

        // Audio and video tracks are collected now
        assert(client->video.has_value() && client->audio.has_value());
        auto video = client->video.value();

        if (avStream.has_value()) {
            sendInitialNalus(avStream.value(), video);
        }

        client->setState(Client::State::Ready);
    }
    if (client->getState() == Client::State::Ready) {
        startStream();
    }
}
