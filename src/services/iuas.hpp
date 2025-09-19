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

/// Request service provider to circle a cooridinate
auto pointOrbit(std::shared_ptr<mavsdk::Telemetry> telemetry, std::shared_ptr<mavsdk::System> system, std::shared_ptr<ndn::Scheduler> scheduler, std::function<void()> offboard_orbit) {
    auto pointOrbitHandler = [&, offboard_orbit](const ndn::Name& requesterIdentity, const muas::IUAS_PointOrbit_Request& _request, muas::IUAS_PointOrbit_Response& _response){
        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = set_request_ts(time_req_sent);
        auto passthrough = mavsdk::MavlinkPassthrough{system};

        NDN_LOG_INFO("PointOrbit request received");
        NDN_LOG_INFO("PointOrbit request latency: " << req_latency_ms << " ms");

        // If UAS is grounded, ignore
        if (!telemetry->in_air()) {
            NDN_LOG_INFO("PointOrbit request denied: IUAS has not taken off");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("IUAS has not taken off");
            set_response_ts(time_req_recv, _response);
            return;
        }

        // Read position variables from request message
        auto pos = _request.target();
        auto latitude = pos.latitude();
        auto longitude = pos.longitude();
        auto altitude = pos.altitude();
        
        // Hard-coded orbit parameters
        float num_turns = 3.0; // default number of turns
        float orbit_radius = 2.0;  // default radius
        float orbit_velocity = 0.5;  // default velocity

        // Attempting to use MAVLink Passthrough to engage CIRCLE mode on autopilot
        // POSSIBLE CAUSE FOR LANDING BEHAVIOR (LOGS SHOW UAS CHANGED TO LOITER MODE)
        mavsdk::MavlinkPassthrough::CommandLong command_long{};
        command_long.command = MAV_CMD_NAV_LOITER_TURNS;
        command_long.target_sysid = passthrough.get_target_sysid();
        command_long.target_compid = passthrough.get_target_compid();
        command_long.param1 = num_turns;
        command_long.param3 = orbit_radius;
        command_long.param5 = latitude;
        command_long.param6 = longitude;
        command_long.param7 = 0.0f; // Use current altitude

        // Send the command to autopilot via passthrough
        const mavsdk::MavlinkPassthrough::Result orbit_result = passthrough.send_command_long(
            command_long
        );

        // If command fails, use 'offboard_orbit' anonymous function
        if (orbit_result != mavsdk::MavlinkPassthrough::Result::Success) {
            NDN_LOG_INFO("PointOrbit request failed: " << orbit_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Orbit failed. Attempting offboard control.");
            std::cout << "Beginning offboard orbit in 3 seconds." << std::endl;
            scheduler->schedule(ndn::time::milliseconds(3000), offboard_orbit);
            set_response_ts(time_req_recv, _response);
            return;
        }
        
        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Beginning orbit routine at target position");
        set_response_ts(time_req_recv, _response);
    };

    return pointOrbitHandler;
}