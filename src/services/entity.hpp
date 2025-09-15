#include <sys/time.h>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <ctype.h>

#include "../util/latency.hpp"

auto echo() {
    auto echoHandler = [](const ndn::Name& requesterIdentity, const muas::Entity_Echo_Request& _request, muas::Entity_Echo_Response& _response){
        auto set_response = [&](const google::protobuf::Timestamp& time_req_recv) {
            struct timeval tv;
            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
        };

        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = request_ts_init(time_req_sent);

        NDN_LOG_INFO("Echo request received");

        NDN_LOG_INFO("Echo request latency: " << req_latency_ms << " ms");

        set_response(time_req_recv);
    };

    return echoHandler;
}