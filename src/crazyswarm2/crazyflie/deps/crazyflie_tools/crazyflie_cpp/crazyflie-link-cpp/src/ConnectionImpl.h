#pragma once

#include <atomic>
#include <chrono>

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>

#include "Crazyradio.h"
#include "CrazyflieUSB.h"
#include "crazyflieLinkCpp/Connection.h"

namespace bitcraze {
namespace crazyflieLinkCpp {

class ConnectionImpl
{
public:
    std::string uri_;
    int devid_;

    Connection::Statistics statistics_;
    Connection::Statistics statistics_previous_;

    bool isRadio_;

    // Radio related
    int channel_;
    Crazyradio::Datarate datarate_;
    uint64_t address_;
    bool useSafelink_;
    bool useAutoPing_;
    bool useAckFilter_;
    //: Packet tracing for ONE connection, default off. Added 2026-10-05 to find
    //: the "link alive, log data dead" stall: a drone stops delivering every log
    //: block while still answering polls and accepting commands. Off = one
    //: relaxed atomic load per ack, which changes no behaviour and no timing.
    std::atomic<bool> trace_{false};
    //: Trace accounting, printed once a second rather than per packet: a healthy
    //: drone returns ~165 acks/s, so per-packet printing from the radio thread
    //: would be ~825 lines/s across five drones and would perturb the timing we
    //: are trying to measure.
    uint32_t trace_null_{0};
    uint32_t trace_log_{0};
    uint32_t trace_other_{0};
    std::chrono::steady_clock::time_point trace_tick_{};
    bool safelinkInitialized_;
    bool safelinkDown_;
    bool safelinkUp_;
    bool broadcast_;

    std::mutex queue_send_mutex_;
    std::priority_queue<Packet, std::vector<Packet>, std::greater<Packet>> queue_send_;
    Packet retry_;

    std::mutex queue_recv_mutex_;
    std::condition_variable queue_recv_cv_;
    std::priority_queue<Packet, std::vector<Packet>, std::greater<Packet>> queue_recv_;

    std::string runtime_error_;
};

} // namespace crazyflieLinkCpp
} // namespace bitcraze