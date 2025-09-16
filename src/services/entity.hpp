#include <sys/time.h>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <ctype.h>

#include "../util/latency.hpp"

/// Request a provider to respond with a timestamp for RTT calculation
auto echo() {
    auto echoHandler = [](const ndn::Name& requesterIdentity, const muas::Entity_Echo_Request& _request, muas::Entity_Echo_Response& _response){
        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = set_request_ts(time_req_sent);
        NDN_LOG_INFO("Echo request received");
        NDN_LOG_INFO("Echo request latency: " << req_latency_ms << " ms");
        set_response_ts(time_req_recv, _response);
    };

    return echoHandler;
}