#include <iostream>
#include <sstream>
#include <string>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>
#include "./generated/ServiceProvider_IUAS.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <mavsdk/plugins/action/action.h>
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
        auto action = mavsdk::Action{system};

        if (m_telemetry.gps_info().num_satellites < 5) {
            NDN_LOG_INFO("Takeoff request denied: need more than 5 satellites (" << m_telemetry.gps_info().num_satellites << ")");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Not enough satellites");
            return;
        }

        if (m_telemetry.in_air()) {
            NDN_LOG_INFO("Takeoff request denied: Already in the air!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("UAS has already taken off");
            return;
        }

        if (!m_telemetry.armed()) {
            const mavsdk::Action::Result arm_result = action.arm();
            if (arm_result != mavsdk::Action::Result::Success) {
                NDN_LOG_INFO("Arming failed: " << arm_result);
                _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
                _response.mutable_response()->set_msg("Arming failed");
                return;
            }
            NDN_LOG_INFO("Armed");
        }

        const mavsdk::Action::Result takeoff_result = action.takeoff();
        if (takeoff_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Takeoff failed: " << takeoff_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Takeoff failed");
            return;
        }
        
        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Taking off");
    };

    m_serviceProvider.m_FlightCtrlService.Land_Handler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_Land_Request& _request, muas::FlightCtrl_Land_Response& _response){
        auto action = mavsdk::Action{system};

        if (!m_telemetry.in_air()) {
            NDN_LOG_INFO("Land request denied: Already grounded!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Already grounded");
            return;
        }

        const mavsdk::Action::Result land_result = action.land();
        if (land_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Landing failed: " << land_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Landing failed");
            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Landing");
    };

    m_serviceProvider.m_FlightCtrlService.RTL_Handler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_RTL_Request& _request, muas::FlightCtrl_RTL_Response& _response){
        auto action = mavsdk::Action{system};

        if (!m_telemetry.in_air()) {
            NDN_LOG_INFO("RTL request denied: Already grounded!");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Already grounded");
            return;
        }

        const mavsdk::Action::Result rtl_result = action.return_to_launch();
        if (rtl_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("RTL failed: " << rtl_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("RTL failed");
            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Initiating RTL");
    };

    m_serviceProvider.m_FlightCtrlService.Kill_Handler = [&](const ndn::Name& requesterIdentity, const muas::FlightCtrl_Kill_Request& _request, muas::FlightCtrl_Kill_Response& _response){
        auto action = mavsdk::Action{system};

        const mavsdk::Action::Result kill_result = action.kill();
        if (kill_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("Kill command failed: " << kill_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Kill command failed");
            return;
        }

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Killed");
    };

    m_serviceProvider.m_IUASService.PointOrbit_Handler = [&](const ndn::Name& requesterIdentity, const muas::IUAS_PointOrbit_Request& _request, muas::IUAS_PointOrbit_Response& _response){
        auto action = mavsdk::Action{system};

        if (!m_telemetry.in_air()) {
            NDN_LOG_INFO("PointOrbit request denied: IUAS has not taken off");
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("IUAS has not taken off");
            return;
        }

        auto pos = _request.target();
        auto latitude = pos.latitude();
        auto longitude = pos.longitude();
        auto altitude = pos.altitude();

        float orbit_radius = 8.0;  // default radius
        float orbit_velocity = 1.0;  // default velocity

        const mavsdk::Action::Result orbit_result = action.do_orbit(
            orbit_radius,
            orbit_velocity,
            mavsdk::Action::OrbitYawBehavior::HoldFrontToCircleCenter,
            latitude,
            longitude,
            altitude
        );
        if (orbit_result != mavsdk::Action::Result::Success) {
            NDN_LOG_INFO("PointOrbit request failed: " << orbit_result);
            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Orbit failed");
            return;
        }
        
        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Beginning orbit routine at target position");
    };

    m_serviceProvider.m_SensorService.GetSensorInfo_Handler = [&, sensor](const ndn::Name& requesterIdentity, const muas::SensorCtrl_GetSensorInfo_Request& _request, muas::SensorCtrl_GetSensorInfo_Response& _response){
        auto action = mavsdk::Action{system};

        muas::Sensor* s = _response.add_sensors();
        s->set_name(sensor.name());
        s->set_id(sensor.id());
        s->set_type(sensor.type());
        s->set_data_namespace(sensor.data_namespace());

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Sensor info request satisfied.");
    };

    m_serviceProvider.m_SensorService.CaptureSingle_Handler = [&](const ndn::Name& requesterIdentity, const muas::SensorCtrl_CaptureSingle_Request& _request, muas::SensorCtrl_CaptureSingle_Response& _response){
        auto action = mavsdk::Action{system};

        int cam_idx = 0;

        std::cout << "Trying to open camera (/dev/video" << cam_idx << ")..." << std::endl;
        cv::VideoCapture capture(cam_idx);
        while (!capture.isOpened() || cam_idx < 5)
        {
            NDN_LOG_ERROR("ERROR: Can't initialize camera (/dev/video" << cam_idx << ")");
            cam_idx++;
            std::cout << "Trying to open camera (/dev/video" << cam_idx << ")..." << std::endl;
            cv::VideoCapture capture(cam_idx);
        }

        cv::Mat frame;

        capture >> frame;
        const char *directory = "./captures";  // current directory
        int next_num = get_next_file_number(directory);

        char filename[256];
        snprintf(filename, sizeof(filename), "%d.png", next_num);
        cv::imwrite(filename,frame);
        NDN_LOG_INFO("Saved " << filename);

        char msg[200];

        snprintf(msg,sizeof(msg),"Single capture successful. Index: %i", next_num);

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg(msg);
        _response.set_capture_id(std::to_string(next_num));
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