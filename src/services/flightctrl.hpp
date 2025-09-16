#include <iostream>
#include <string>
#include <sys/time.h>
#include <chrono>
#include <thread>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>

#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <ctype.h>

#include "../util/latency.hpp"

using std::chrono::seconds;
using std::this_thread::sleep_for;

auto takeoff(std::shared_ptr<mavsdk::Telemetry> telemetry, std::shared_ptr<mavsdk::System> system) {
    auto takeoffHandler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_Takeoff_Request& _request, muas::FlightCtrl_Takeoff_Response& _response){
        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = set_request_ts(time_req_sent);

        NDN_LOG_INFO("Takeoff request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("Takeoff request latency: " << req_latency_ms << " ms");

        if (telemetry->gps_info().num_satellites < 5) {
            NDN_LOG_INFO("Takeoff request denied: need more than 5 satellites (" << telemetry->gps_info().num_satellites << ")");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Not enough satellites");
            set_response_ts(time_req_recv, _response);
            return;
        }

        if (telemetry->in_air()) {
            NDN_LOG_INFO("Takeoff request denied: Already in the air!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("UAS has already taken off");
            set_response_ts(time_req_recv, _response);
            return;
        }

        if (!telemetry->armed()) {
            const mavsdk::Action::Result arm_result = action.arm();
            if (arm_result != mavsdk::Action::Result::Success) {
                NDN_LOG_INFO("Arming failed: " << arm_result);
                _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
                _response.mutable_response()->set_msg("Arming failed");
                set_response_ts(time_req_recv, _response);
                return;
            }
            NDN_LOG_INFO("Armed");
        }

        const mavsdk::Action::Result takeoff_result = action.takeoff();
        if (takeoff_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Takeoff failed: " << takeoff_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Takeoff failed");
            set_response_ts(time_req_recv, _response);
            return;
        }
        
        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Taking off");
        set_response_ts(time_req_recv, _response);
    };

    return takeoffHandler;
}

auto land(std::shared_ptr<mavsdk::Telemetry> telemetry, std::shared_ptr<mavsdk::System> system) {
    auto landHandler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_Land_Request& _request, muas::FlightCtrl_Land_Response& _response){
        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = set_request_ts(time_req_sent);

        NDN_LOG_INFO("Land request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("Land request latency: " << req_latency_ms << " ms");
        
        if (!telemetry->in_air()) {
            NDN_LOG_INFO("Land request denied: Already grounded!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Already grounded");
            set_response_ts(time_req_recv, _response);
            return;
        }

        const mavsdk::Action::Result land_result = action.land();
        if (land_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Landing failed: " << land_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Landing failed");
            set_response_ts(time_req_recv, _response);
            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Landing");
        set_response_ts(time_req_recv, _response);
    };

    return landHandler;
}

auto rtl(std::shared_ptr<mavsdk::Telemetry> telemetry, std::shared_ptr<mavsdk::System> system) {
    auto rtlHandler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_RTL_Request& _request, muas::FlightCtrl_RTL_Response& _response){
        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = set_request_ts(time_req_sent);

        NDN_LOG_INFO("RTL request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("RTL request latency: " << req_latency_ms << " ms");

        if (!telemetry->in_air()) {
            NDN_LOG_INFO("RTL request denied: Already grounded!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Already grounded");
            set_response_ts(time_req_recv, _response);
            return;
        }

        const mavsdk::Action::Result rtl_result = action.return_to_launch();
        if (rtl_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("RTL failed: " << rtl_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("RTL failed");
            set_response_ts(time_req_recv, _response);
            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Initiating RTL");
        set_response_ts(time_req_recv, _response);
    };

    return rtlHandler;
}

auto kill(std::shared_ptr<mavsdk::Telemetry> telemetry, std::shared_ptr<mavsdk::System> system) {
    auto killHandler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_Kill_Request& _request, muas::FlightCtrl_Kill_Response& _response){
        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = set_request_ts(time_req_sent);

        NDN_LOG_INFO("Kill request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("Kill request latency: " << req_latency_ms << " ms");

        const mavsdk::Action::Result kill_result = action.kill();
        if (kill_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Kill command failed: " << kill_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Kill command failed");
            set_response_ts(time_req_recv, _response);
            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Killed");
        set_response_ts(time_req_recv, _response);
    };

    return killHandler;
}