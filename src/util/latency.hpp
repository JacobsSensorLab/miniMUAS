#ifndef Latency_HPP
#define Latency_HPP

#include <sys/time.h>
#include "./generated/messages.pb.h"

std::pair<long, google::protobuf::Timestamp>
request_ts_init(const google::protobuf::Timestamp& time_req_sent) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    google::protobuf::Timestamp time_req_recv;
    time_req_recv.set_seconds(tv.tv_sec);
    time_req_recv.set_nanos(tv.tv_usec * 1000);

    auto req_recv_ms = (time_req_recv.seconds() * 1000) + (time_req_recv.nanos() / 1000000);
    auto req_sent_ms = (time_req_sent.seconds() * 1000) + (time_req_sent.nanos() / 1000000);

    return {req_recv_ms - req_sent_ms, time_req_recv};
}
#endif