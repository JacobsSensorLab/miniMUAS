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

auto pointOrbit(std::shared_ptr<mavsdk::Telemetry> telemetry, std::shared_ptr<mavsdk::System> system, std::shared_ptr<ndn::Scheduler> scheduler, std::function<void()> offboard_orbit) {
    auto pointOrbitHandler = [&, offboard_orbit](const ndn::Name& requesterIdentity, const muas::IUAS_PointOrbit_Request& _request, muas::IUAS_PointOrbit_Response& _response){
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

        NDN_LOG_INFO("PointOrbit request received");
        // auto action = mavsdk::Action{system};
        auto passthrough = mavsdk::MavlinkPassthrough{system};

        NDN_LOG_INFO("PointOrbit request latency: " << req_latency_ms << " ms");

        if (!telemetry->in_air()) {
            NDN_LOG_INFO("PointOrbit request denied: IUAS has not taken off");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("IUAS has not taken off");
            set_response(time_req_recv);
            return;
        }

        auto pos = _request.target();
        auto latitude = pos.latitude();
        auto longitude = pos.longitude();
        auto altitude = pos.altitude();
        
        float num_turns = 3.0; // default number of turns
        float orbit_radius = 2.0;  // default radius
        float orbit_velocity = 0.5;  // default velocity

        mavsdk::MavlinkPassthrough::CommandLong command_long{};
        command_long.command = MAV_CMD_NAV_LOITER_TURNS;
        command_long.target_sysid = passthrough.get_target_sysid();
        command_long.target_compid = passthrough.get_target_compid();
        command_long.param1 = num_turns;
        command_long.param3 = orbit_radius;
        command_long.param5 = latitude;
        command_long.param6 = longitude;
        command_long.param7 = 0.0f; // Use current altitude

        const mavsdk::MavlinkPassthrough::Result orbit_result = passthrough.send_command_long(
            command_long
        );

        if (orbit_result != mavsdk::MavlinkPassthrough::Result::Success) {
            NDN_LOG_INFO("PointOrbit request failed: " << orbit_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Orbit failed. Attempting offboard control.");
            std::cout << "Beginning offboard orbit in 3 seconds." << std::endl;
            scheduler->schedule(ndn::time::milliseconds(3000), offboard_orbit);
            set_response(time_req_recv);
            return;
        }
        
        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Beginning orbit routine at target position");
        set_response(time_req_recv);
    };

    return pointOrbitHandler;
}