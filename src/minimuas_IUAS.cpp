#include <iostream>
#include <string>
#include <sys/time.h>
#include <chrono>
#include <thread>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>
#include "./generated/ServiceProvider_IUAS.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <ctype.h>

NDN_LOG_INIT(muas.iuas_drone);

#include "./services/flightctrl.hpp"
#include "./services/entity.hpp"
#include "./services/sensor.hpp"
#include "./services/iuas.hpp"

using std::chrono::seconds;
using std::this_thread::sleep_for;

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

    // Configure autopilot connection details
    mavsdk::Mavsdk mav{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer}};
    mavsdk::ConnectionResult connection_result = mav.add_any_connection(argv[2]);

    // Only start app if autopilot connection is successful
    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    // Trying to get a handle for the autopilot
    auto opt_system = mav.first_autopilot(-1);
    if (!opt_system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Unwrapping the autopilot from the optional
    auto system = opt_system.value();
    
    // Instantiating a global telemetry object using MAVSDK Telemtry plugin
    auto m_telemetry = std::make_shared<mavsdk::Telemetry>(system);

    // Setting refresh rates for desired parameters; must be done
    m_telemetry->set_rate_in_air(0.5);
    m_telemetry->set_rate_gps_info(0.5);

    // Instantiating an offboard instance using MAVSDK Offboard plugin
    auto m_offboard = mavsdk::Offboard{system};

    std::string identity = argv[1];
    std::string conf_dir = "/usr/local/bin";

    // Defining hard-coded sensor parameters
    auto sensor_idx = 0;
    muas::Sensor sensor;
    std::string sensor_namespace = identity + "/sensor/" + std::to_string(sensor_idx);
    sensor.set_name("IUAS_Arducam");
    sensor.set_type(muas::Sensor_SensorType_MULTISPECTRAL);
    sensor.set_id(sensor_idx);
    sensor.set_data_namespace(sensor_namespace);

    // Same as minimuas_GCS_shell
    ndn::Face m_face;
    auto m_scheduler = std::make_shared<ndn::Scheduler>(m_face.getIoContext());
    ndn::security::KeyChain m_keyChain;
    ndn::security::Certificate gs_certificate(
        m_keyChain.getPib()
        .getIdentity(identity)
        .getDefaultKey()
        .getDefaultCertificate()
    );

    // Instantiating the IUAS service provider to listen and serve requests
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

    // A hard-coded routine to tell UAS to fly to a coordinate and orbit it
    // Relies on MAVSDK Offboard plugin
    // TODO: fix issue where UAS lands instead of orbit point
    auto offboard_orbit = [&]() {
        NDN_LOG_INFO("Reading home position in Global coordinates");

        const auto res_and_gps_origin = m_telemetry->get_gps_global_origin();
        if (res_and_gps_origin.first != mavsdk::Telemetry::Result::Success) {
            std::cerr << "Telemetry failed: " << res_and_gps_origin.first << '\n';
        }
        mavsdk::Telemetry::GpsGlobalOrigin origin = res_and_gps_origin.second;
        std::cerr << "Origin (lat, lon, alt amsl):\n " << origin << '\n';

        NDN_LOG_INFO("Starting Offboard position control in Global coordinates");
        
        // Send it once before starting offboard, otherwise it will be rejected.
        // this goes to the center of the ellipse, using the default altitude type (altitude relative to home)
        const mavsdk::Offboard::PositionGlobalYaw north{
            35.120881,
            -89.934772,
            6.0f,
            0.0f
        };
        m_offboard.set_position_global(north);

        mavsdk::Offboard::Result offboard_result = m_offboard.start();
        if (offboard_result != mavsdk::Offboard::Result::Success) {
            NDN_LOG_INFO("Offboard start failed: " << offboard_result);
        }

        NDN_LOG_INFO("Offboard started");
        NDN_LOG_INFO("Going to coordinate near center of ellipse");
        sleep_for(seconds(10));

        offboard_result = m_offboard.stop();
        if (offboard_result != mavsdk::Offboard::Result::Success) {
            std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        }

        NDN_LOG_INFO("Wait for a bit");
        mavsdk::Offboard::VelocityBodyYawspeed stay{};
        m_offboard.set_velocity_body(stay);
        sleep_for(seconds(2));
        
        NDN_LOG_INFO("Fly a circle sideways");
        mavsdk::Offboard::VelocityBodyYawspeed circle{};
        circle.right_m_s = -5.0f;
        circle.yawspeed_deg_s = 30.0f;
        m_offboard.set_velocity_body(circle);
        sleep_for(seconds(15));

        NDN_LOG_INFO("Wait for a bit");
        m_offboard.set_velocity_body(stay);
        sleep_for(seconds(2));

        offboard_result = m_offboard.stop();
        if (offboard_result != mavsdk::Offboard::Result::Success) {
            NDN_LOG_INFO("Offboard stop failed: " << offboard_result);
        }
        NDN_LOG_INFO("Offboard stopped");
    };

    // Assigning the appropriate service request handlers; handlers are defined in the services directory
    m_serviceProvider.m_FlightCtrlService.Takeoff_Handler = takeoff(m_telemetry, system);
    m_serviceProvider.m_FlightCtrlService.Land_Handler = land(m_telemetry, system);
    m_serviceProvider.m_FlightCtrlService.RTL_Handler = rtl(m_telemetry, system);
    m_serviceProvider.m_FlightCtrlService.Kill_Handler = kill(m_telemetry, system);
    m_serviceProvider.m_IUASService.PointOrbit_Handler = pointOrbit(m_telemetry, system, m_scheduler, offboard_orbit);
    m_serviceProvider.m_SensorService.GetSensorInfo_Handler = getSensorInfo(sensor);
    m_serviceProvider.m_SensorService.CaptureSingle_Handler = captureSingle();
    m_serviceProvider.m_EntityService.Echo_Handler = echo();

    NDN_LOG_INFO("IUAS running");
    try {
        while (1) {
            m_face.processEvents(ndn::time::milliseconds(0),true); // main event loop runs until terminated
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}