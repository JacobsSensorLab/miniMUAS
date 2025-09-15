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

NDN_LOG_INIT(muas.wuas_drone);

#include "./services/flightctrl.hpp"
#include "./services/entity.hpp"
#include "./services/wuas.hpp"

int
main(int argc, char **argv)
{
    auto takeoff_metric = std::make_shared<Metrics>(true, true);
    auto orbit_metric = std::make_shared<Metrics>(true, true);

    bool single_request_sent = false;

    if (argc != 3)
    {
        std::cerr << "Usage : wuas-example <identity> <connection_url>\n"
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
    
    auto m_telemetry = std::make_shared<mavsdk::Telemetry>(system);
    m_telemetry->set_rate_in_air(0.5);
    m_telemetry->set_rate_gps_info(0.5);

    std::string identity = argv[1];
    std::string conf_dir = "/usr/local/bin";
    ndn::Face m_face;
    ndn::Scheduler m_scheduler(m_face.getIoContext());
    ndn::security::KeyChain m_keyChain;
    ndn::security::Certificate wuas_certificate(
        m_keyChain.getPib()
        .getIdentity(identity)
        .getDefaultKey()
        .getDefaultCertificate()
    );

    muas::Sensor sensor;
    char sensor_namespace[200];
    snprintf(sensor_namespace, sizeof(sensor_namespace), "/muas/%s/sensor/0", identity.c_str());
    sensor.set_name("WUAS_Arducam");
    sensor.set_type(muas::Sensor_SensorType_MULTISPECTRAL);
    sensor.set_id(0);
    sensor.set_data_namespace(sensor_namespace);

    muas::ServiceProvider_WUAS m_serviceProvider(
          m_face
        , "/muas"
        , wuas_certificate
        , m_keyChain.getPib()
            .getIdentity("/muas/aa")
            .getDefaultKey()
            .getDefaultCertificate()
        , conf_dir + "/trust-any.conf"
    );

    auto m_serviceUser = std::make_shared<muas::ServiceUser_WUAS>(
          m_face
        , "/muas"
        , wuas_certificate
        , m_keyChain
            .getPib()
            .getIdentity("/muas/aa")
            .getDefaultKey()
            .getDefaultCertificate()
        , conf_dir + "/trust-any.conf"
    );

    auto interrogate = [&]() {
        std::cout << "Beginning interrogation." << std::endl;
        m_scheduler.schedule(ndn::time::milliseconds(0), takeoff(m_serviceUser, takeoff_metric));
        m_scheduler.schedule(ndn::time::milliseconds(5000), orbit(m_serviceUser, orbit_metric));
    };

    auto OutputMetrics = [&]() {
        takeoff_metric->printStats();
        takeoff_metric->exportCSV("wuas_takeoff.csv");
        orbit_metric->printStats();
        orbit_metric->exportCSV("wuas_orbit.csv");
    };

    m_serviceProvider.m_FlightCtrlService.Takeoff_Handler = takeoff(m_telemetry, system);
    m_serviceProvider.m_FlightCtrlService.Land_Handler = land(m_telemetry, system);
    m_serviceProvider.m_FlightCtrlService.RTL_Handler = rtl(m_telemetry, system);
    m_serviceProvider.m_FlightCtrlService.Kill_Handler = kill(m_telemetry, system);
    m_serviceProvider.m_EntityService.Echo_Handler = echo();

    NDN_LOG_INFO("WUAS running");
    try {
        while (1) {
            if (m_telemetry->in_air() && !single_request_sent) {
                std::cout << "Beginning interrogation in 10 seconds." << std::endl;
                m_scheduler.schedule(ndn::time::milliseconds(10000), interrogate);
                m_scheduler.schedule(ndn::time::milliseconds(25000), OutputMetrics);
                single_request_sent = true;
            }
            m_face.processEvents(ndn::time::milliseconds(-1),true);
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}