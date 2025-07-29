#include <iostream>
#include <string>
#include <sys/time.h>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>
#include "./generated/ServiceProvider_IUAS.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <ctype.h>

NDN_LOG_INIT(muas.iuas_drone);

int get_next_file_number(const char *directory) {
    DIR *dir;
    struct dirent *entry;
    int max_num = -1;

    dir = opendir(directory);
    if (dir == NULL) {
        perror("opendir");
        return 0;
    }

    while ((entry = readdir(dir)) != NULL) {
        const char *filename = entry->d_name;
        int len = strlen(filename);
        if (len > 4 && strcmp(filename + len - 4, ".png") == 0) {
            char num_part[256];
            strncpy(num_part, filename, len - 4);
            num_part[len - 4] = '\0';

            // Check if it's all digits
            int is_number = 1;
            for (int i = 0; num_part[i] != '\0'; i++) {
                if (!isdigit((unsigned char)num_part[i])) {
                    is_number = 0;
                    break;
                }
            }

            if (is_number) {
                int num = atoi(num_part);
                if (num > max_num) {
                    max_num = num;
                }
            }
        }
    }

    closedir(dir);
    return max_num + 1;
}

int
main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage : iuas-example <identity> <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
        exit(1);
    }

    mavsdk::Mavsdk mav{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer}};
    mavsdk::ConnectionResult connection_result = mav.add_any_connection(argv[2]);

    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto opt_system = mav.first_autopilot(-1);
    if (!opt_system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    auto system = opt_system.value();
    
    auto m_telemetry = mavsdk::Telemetry{system};
    m_telemetry.set_rate_in_air(0.5);
    m_telemetry.set_rate_gps_info(0.5);

    std::string identity = argv[1];
    std::string conf_dir = "/usr/local/bin";
    ndn::Face m_face;
    auto sensor_idx = 0;
    muas::Sensor sensor;
    std::string sensor_namespace = identity + "/sensor/" + std::to_string(sensor_idx);
    sensor.set_name("IUAS_Arducam");
    sensor.set_type(muas::Sensor_SensorType_MULTISPECTRAL);
    sensor.set_id(sensor_idx);
    sensor.set_data_namespace(sensor_namespace);

    ndn::security::KeyChain m_keyChain;
    ndn::security::Certificate gs_certificate(
        m_keyChain.getPib()
        .getIdentity(identity)
        .getDefaultKey()
        .getDefaultCertificate()
    );
    muas::ServiceProvider_IUAS m_serviceProvider(
          m_face
        , "/muas"
        , gs_certificate
        , m_keyChain.getPib()
            .getIdentity("/muas/aa")
            .getDefaultKey()
            .getDefaultCertificate()
        , conf_dir + "/trust-any.conf"
    );

    m_serviceProvider.m_FlightCtrlService.Takeoff_Handler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_Takeoff_Request& _request, muas::FlightCtrl_Takeoff_Response& _response){
        struct timeval tv;
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_req_recv;
        time_req_recv.set_seconds(tv.tv_sec);
        time_req_recv.set_nanos(tv.tv_usec * 1000);

        auto time_req_sent = _request.time_request_sent();

        auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
        auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
        auto req_latency_ms = req_recv_ms - req_sent_ms;

        NDN_LOG_INFO("Takeoff request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("Takeoff request latency: " << req_latency_ms << " ms");

        if (m_telemetry.gps_info().num_satellites < 5) {
            NDN_LOG_INFO("Takeoff request denied: need more than 5 satellites (" << m_telemetry.gps_info().num_satellites << ")");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Not enough satellites");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }

        if (m_telemetry.in_air()) {
            NDN_LOG_INFO("Takeoff request denied: Already in the air!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("UAS has already taken off");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }

        if (!m_telemetry.armed()) {
            const mavsdk::Action::Result arm_result = action.arm();
            if (arm_result != mavsdk::Action::Result::Success) {
                NDN_LOG_INFO("Arming failed: " << arm_result);
                _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
                _response.mutable_response()->set_msg("Arming failed");

                gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

                return;
            }
            NDN_LOG_INFO("Armed");
        }

        const mavsdk::Action::Result takeoff_result = action.takeoff();
        if (takeoff_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Takeoff failed: " << takeoff_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Takeoff failed");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }
        
        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Taking off");
        
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_res_sent;
        time_res_sent.set_seconds(tv.tv_sec);
        time_res_sent.set_nanos(tv.tv_usec * 1000);

        _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
        _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
        _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
        _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
    };

    m_serviceProvider.m_FlightCtrlService.Land_Handler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_Land_Request& _request, muas::FlightCtrl_Land_Response& _response){
        struct timeval tv;
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_req_recv;
        time_req_recv.set_seconds(tv.tv_sec);
        time_req_recv.set_nanos(tv.tv_usec * 1000);

        auto time_req_sent = _request.time_request_sent();

        auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
        auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
        auto req_latency_ms = req_recv_ms - req_sent_ms;

        NDN_LOG_INFO("Land request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("Land request latency: " << req_latency_ms << " ms");
        
        if (!m_telemetry.in_air()) {
            NDN_LOG_INFO("Land request denied: Already grounded!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Already grounded");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }

        const mavsdk::Action::Result land_result = action.land();
        if (land_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Landing failed: " << land_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Landing failed");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Landing");

        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_res_sent;
        time_res_sent.set_seconds(tv.tv_sec);
        time_res_sent.set_nanos(tv.tv_usec * 1000);

        _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
        _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
        _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
        _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
    };

    m_serviceProvider.m_FlightCtrlService.RTL_Handler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_RTL_Request& _request, muas::FlightCtrl_RTL_Response& _response){
        struct timeval tv;
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_req_recv;
        time_req_recv.set_seconds(tv.tv_sec);
        time_req_recv.set_nanos(tv.tv_usec * 1000);

        auto time_req_sent = _request.time_request_sent();

        auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
        auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
        auto req_latency_ms = req_recv_ms - req_sent_ms;

        NDN_LOG_INFO("RTL request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("RTL request latency: " << req_latency_ms << " ms");

        if (!m_telemetry.in_air()) {
            NDN_LOG_INFO("RTL request denied: Already grounded!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Already grounded");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }

        const mavsdk::Action::Result rtl_result = action.return_to_launch();
        if (rtl_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("RTL failed: " << rtl_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("RTL failed");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Initiating RTL");

        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_res_sent;
        time_res_sent.set_seconds(tv.tv_sec);
        time_res_sent.set_nanos(tv.tv_usec * 1000);

        _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
        _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
        _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
        _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
    };

    m_serviceProvider.m_FlightCtrlService.Kill_Handler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_Kill_Request& _request, muas::FlightCtrl_Kill_Response& _response){
        struct timeval tv;
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_req_recv;
        time_req_recv.set_seconds(tv.tv_sec);
        time_req_recv.set_nanos(tv.tv_usec * 1000);

        auto time_req_sent = _request.time_request_sent();

        auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
        auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
        auto req_latency_ms = req_recv_ms - req_sent_ms;

        NDN_LOG_INFO("Kill request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("Kill request latency: " << req_latency_ms << " ms");

        const mavsdk::Action::Result kill_result = action.kill();
        if (kill_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Kill command failed: " << kill_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Kill command failed");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Killed");

        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_res_sent;
        time_res_sent.set_seconds(tv.tv_sec);
        time_res_sent.set_nanos(tv.tv_usec * 1000);

        _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
        _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
        _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
        _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
    };

    m_serviceProvider.m_IUASService.PointOrbit_Handler = [&](const ndn::Name& requesterIdentity, const muas::IUAS_PointOrbit_Request& _request, muas::IUAS_PointOrbit_Response& _response){
        struct timeval tv;
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_req_recv;
        time_req_recv.set_seconds(tv.tv_sec);
        time_req_recv.set_nanos(tv.tv_usec * 1000);

        auto time_req_sent = _request.time_request_sent();

        auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
        auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
        auto req_latency_ms = req_recv_ms - req_sent_ms;

        NDN_LOG_INFO("PointOrbit request received");
        // auto action = mavsdk::Action{system};
        auto passthrough = mavsdk::MavlinkPassthrough{system};

        NDN_LOG_INFO("PointOrbit request latency: " << req_latency_ms << " ms");

        if (!m_telemetry.in_air()) {
            NDN_LOG_INFO("PointOrbit request denied: IUAS has not taken off");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("IUAS has not taken off");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

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
            _response.mutable_response()->set_msg("Orbit failed");

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());

            return;
        }
        
        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Beginning orbit routine at target position");

        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_res_sent;
        time_res_sent.set_seconds(tv.tv_sec);
        time_res_sent.set_nanos(tv.tv_usec * 1000);

        _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
        _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
        _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
        _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
    };

    m_serviceProvider.m_SensorService.GetSensorInfo_Handler = [&, sensor](const ndn::Name& requesterIdentity, const muas::SensorCtrl_GetSensorInfo_Request& _request, muas::SensorCtrl_GetSensorInfo_Response& _response){
        struct timeval tv;
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_req_recv;
        time_req_recv.set_seconds(tv.tv_sec);
        time_req_recv.set_nanos(tv.tv_usec * 1000);

        auto time_req_sent = _request.time_request_sent();

        auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
        auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
        auto req_latency_ms = req_recv_ms - req_sent_ms;
        
        NDN_LOG_INFO("SensorInfo request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("SensorInfo request latency: " << req_latency_ms << " ms");

        muas::Sensor* s = _response.add_sensors();
        s->set_name(sensor.name());
        s->set_id(sensor.id());
        s->set_type(sensor.type());
        s->set_data_namespace(sensor.data_namespace());

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Sensor info request satisfied.");

        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_res_sent;
        time_res_sent.set_seconds(tv.tv_sec);
        time_res_sent.set_nanos(tv.tv_usec * 1000);

        _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
        _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
        _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
        _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
    };

    m_serviceProvider.m_SensorService.CaptureSingle_Handler = [&](const ndn::Name& requesterIdentity, const muas::SensorCtrl_CaptureSingle_Request& _request, muas::SensorCtrl_CaptureSingle_Response& _response){
        struct timeval tv;
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_req_recv;
        time_req_recv.set_seconds(tv.tv_sec);
        time_req_recv.set_nanos(tv.tv_usec * 1000);

        auto time_req_sent = _request.time_request_sent();

        auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
        auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
        auto req_latency_ms = req_recv_ms - req_sent_ms;
        
        NDN_LOG_INFO("CaptureSingle request received");
        auto action = mavsdk::Action{system};

        NDN_LOG_INFO("CaptureSingle request latency: " << req_latency_ms << " ms");

        int cam_idx = 0;
        std::string cap_dev = "v4l2:///dev/video";
        std::string cap_str = cap_dev + std::to_string(cam_idx);

        std::cout << "Trying to open camera (" << cap_str << ")..." << std::endl;
        cv::VideoCapture capture(cap_str, cv::CAP_V4L2);
        if (!capture.isOpened())
        {
            NDN_LOG_ERROR("ERROR: Can't initialize camera (" << cam_idx << ")");

            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Camera failed to initialize");
            _response.set_capture_id(std::to_string(-1));

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
        }
        else
        {
            cv::Mat frame;

            capture >> frame;
            const char *directory = "./captures";  // current directory
            int next_num = get_next_file_number(directory);

            char filename[256];
            snprintf(filename, sizeof(filename), "%s/%d.png", directory, next_num);
            cv::imwrite(filename,frame);
            NDN_LOG_INFO("Saved " << filename);

            char msg[200];

            snprintf(msg,sizeof(msg),"Single capture successful. Index: %i", next_num);

            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
            _response.mutable_response()->set_msg(msg);
            _response.set_capture_id(std::to_string(next_num));

            gettimeofday(&tv, NULL);

            google::protobuf::Timestamp time_res_sent;
            time_res_sent.set_seconds(tv.tv_sec);
            time_res_sent.set_nanos(tv.tv_usec * 1000);

            _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
            _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
            _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
            _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
        }
    };

    m_serviceProvider.m_EntityService.Echo_Handler = [&](const ndn::Name& requesterIdentity, const muas::Entity_Echo_Request& _request, muas::Entity_Echo_Response& _response){
        struct timeval tv;
        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_req_recv;
        time_req_recv.set_seconds(tv.tv_sec);
        time_req_recv.set_nanos(tv.tv_usec * 1000);

        auto time_req_sent = _request.time_request_sent();

        auto req_recv_ms = (time_req_recv.seconds()*1000) + (time_req_recv.nanos()/1000000);
        auto req_sent_ms = (time_req_sent.seconds()*1000) + (time_req_sent.nanos()/1000000);
        auto req_latency_ms = req_recv_ms - req_sent_ms;

        NDN_LOG_INFO("Echo request received");

        NDN_LOG_INFO("Echo request latency: " << req_latency_ms << " ms");

        gettimeofday(&tv, NULL);

        google::protobuf::Timestamp time_res_sent;
        time_res_sent.set_seconds(tv.tv_sec);
        time_res_sent.set_nanos(tv.tv_usec * 1000);

        _response.mutable_time_request_received()->set_seconds(time_req_recv.seconds());
        _response.mutable_time_request_received()->set_nanos(time_req_recv.nanos());
        _response.mutable_time_response_sent()->set_seconds(time_res_sent.seconds());
        _response.mutable_time_response_sent()->set_nanos(time_res_sent.nanos());
    };

    NDN_LOG_INFO("IUAS running");
    try {
        while (1) {
            m_face.processEvents(ndn::time::milliseconds(0),true);
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}