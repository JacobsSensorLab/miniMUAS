#ifndef Latency_HPP
#define Latency_HPP

#include <sys/time.h>
#include "./generated/messages.pb.h"
#include <ndn-service-framework/NDNSFMessages.hpp>

std::pair<long, google::protobuf::Timestamp>
set_request_ts(const google::protobuf::Timestamp& time_req_sent) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    google::protobuf::Timestamp time_req_recv;
    time_req_recv.set_seconds(tv.tv_sec);
    time_req_recv.set_nanos(tv.tv_usec * 1000);

    auto req_recv_ms = (time_req_recv.seconds() * 1000) + (time_req_recv.nanos() / 1000000);
    auto req_sent_ms = (time_req_sent.seconds() * 1000) + (time_req_sent.nanos() / 1000000);

    return {req_recv_ms - req_sent_ms, time_req_recv};
}

template <typename ResponseT>
void set_response_ts(const google::protobuf::Timestamp& time_req_recv, ResponseT& response)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    google::protobuf::Timestamp time_res_sent;
    time_res_sent.set_seconds(tv.tv_sec);
    time_res_sent.set_nanos(tv.tv_usec * 1000);

    // Assumes ResponseT has the required mutable_*() methods
    response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
    response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
    response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
    response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
}
#endif