#include <iostream>
#include <string>
#include <sys/time.h>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>
#include "./generated/ServiceUser_WUAS.hpp"
#include "./generated/ServiceProvider_WUAS.hpp"
#include "./metrics.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

std::function<void()> takeoff(std::shared_ptr<muas::ServiceUser_WUAS> serviceUser, std::shared_ptr<Metrics> takeoff_metric) {
    auto takeoffRoutine = [&]() {
        struct timeval tv;
        std::vector<ndn::Name> providers;
        providers.push_back(ndn::Name("/muas/iuas-01"));
        muas::FlightCtrl_Takeoff_Request takeoff_request;

        google::protobuf::Timestamp time_req_sent;
        gettimeofday(&tv, NULL);
        time_req_sent.set_seconds(tv.tv_sec);
        time_req_sent.set_nanos(tv.tv_usec * 1000);
        takeoff_request.mutable_time_request_sent()->CopyFrom(time_req_sent);

        auto takeoff_start = takeoff_metric->start();
        serviceUser->Takeoff_Async(providers, takeoff_request, [=, &takeoff_start](const muas::FlightCtrl_Takeoff_Response& _response) {
            takeoff_metric->end(takeoff_start, true);
            NDN_LOG_INFO(_response.DebugString());

            struct timeval tv;
            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_recv;
            time_res_recv.set_seconds(tv.tv_sec);
            time_res_recv.set_nanos(tv.tv_usec * 1000);
            auto time_req_recv = _response.time_request_received();
            auto time_res_sent = _response.time_response_sent();

            auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
                auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
                auto req_latency_ms = req_recv_ms - req_sent_ms;

                auto res_recv_ms = (time_res_recv.seconds()*1000) + (time_res_recv.nanos()/1000000);
                auto res_sent_ms = (time_res_sent.seconds()*1000) + (time_res_sent.nanos()/1000000);
                auto res_latency_ms = res_recv_ms - res_sent_ms;

            NDN_LOG_INFO("Request latency: " << req_latency_ms << " ms / Response latency: " << res_latency_ms << " ms");
            },
            [&](const muas::FlightCtrl_Takeoff_Request& _request) {
                NDN_LOG_INFO("Timeout " << _request.DebugString());
            },
            3000,
            ndn_service_framework::tlv::NoCoordination
        );
    };

    return takeoffRoutine;
}

std::function<void()> orbit(std::shared_ptr<muas::ServiceUser_WUAS> serviceUser, std::shared_ptr<Metrics> orbit_metric) {
    auto orbitRoutine = [&]() {
        struct timeval tv;
        std::vector<ndn::Name> providers;
        providers.push_back(ndn::Name("/muas/iuas-01"));

        muas::IUAS_PointOrbit_Request orbit_request;
        
        auto point = orbit_request.target();
        point.set_altitude(6);
        // Some location in Switzerland bc QGC starts there apparently
        // point.set_latitude(47.397202);
        // point.set_longitude(8.543931);
        point.set_latitude(35.120881);
        point.set_longitude(-89.934772);

        google::protobuf::Timestamp time_req_sent;
        gettimeofday(&tv, NULL);
        time_req_sent.set_seconds(tv.tv_sec);
        time_req_sent.set_nanos(tv.tv_usec * 1000);
        orbit_request.mutable_time_request_sent()->CopyFrom(time_req_sent);
        
        auto orbit_start = orbit_metric->start();
        serviceUser->PointOrbit_Async(providers, orbit_request, [=, &orbit_start](const muas::IUAS_PointOrbit_Response& _response) {
            orbit_metric->end(orbit_start, true);
            NDN_LOG_INFO(_response.DebugString());

            struct timeval tv;
            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_recv;
            time_res_recv.set_seconds(tv.tv_sec);
            time_res_recv.set_nanos(tv.tv_usec * 1000);
            auto time_req_recv = _response.time_request_received();
            auto time_res_sent = _response.time_response_sent();

            auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
            auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
            auto req_latency_ms = req_recv_ms - req_sent_ms;

            auto res_recv_ms = (time_res_recv.seconds()*1000) + (time_res_recv.nanos()/1000000);
            auto res_sent_ms = (time_res_sent.seconds()*1000) + (time_res_sent.nanos()/1000000);
            auto res_latency_ms = res_recv_ms - res_sent_ms;

            NDN_LOG_INFO("Request latency: " << req_latency_ms << " ms / Response latency: " << res_latency_ms << " ms");
            if (_response.response().code() == muas::NDNSF_Response_miniMUAS_Code_SUCCESS) {
                NDN_LOG_INFO("IUAS Point Orbit successfully initialized.");
            }
        },
        [&](const muas::IUAS_PointOrbit_Request& _request) {
            NDN_LOG_INFO("Timeout " << _request.DebugString());
        },
        3000,
        ndn_service_framework::tlv::NoCoordination
        );
    };

    return orbitRoutine;
}