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

int
main(int argc, char **argv)
{
    Metrics takeoff_metric(true, true);
    Metrics orbit_metric(true, true);

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
    
    auto m_telemetry = mavsdk::Telemetry{system};
    m_telemetry.set_rate_in_air(0.5);
    m_telemetry.set_rate_gps_info(0.5);

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

    muas::ServiceUser_WUAS m_serviceUser(
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

    auto takeoff = [&]() {
        struct timeval tv;
        std::vector<ndn::Name> providers;
        providers.push_back(ndn::Name("/muas/iuas-01"));
        muas::FlightCtrl_Takeoff_Request takeoff_request;

        google::protobuf::Timestamp time_req_sent;
        gettimeofday(&tv, NULL);
        time_req_sent.set_seconds(tv.tv_sec);
        time_req_sent.set_nanos(tv.tv_usec * 1000);
        takeoff_request.mutable_time_request_sent()->CopyFrom(time_req_sent);

        auto takeoff_start = takeoff_metric.start();
        m_serviceUser.Takeoff_Async(providers, takeoff_request, [&, takeoff_start, time_req_sent](const muas::FlightCtrl_Takeoff_Response& _response) {
            takeoff_metric.end(takeoff_start, true);
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

    auto orbit = [&]() {
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
        
        auto orbit_start = orbit_metric.start();
        m_serviceUser.PointOrbit_Async(providers, orbit_request, [&, orbit_start, time_req_sent](const muas::IUAS_PointOrbit_Response& _response) {
            orbit_metric.end(orbit_start, true);
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

    auto interrogate = [&]() {
        std::cout << "Beginning interrogation." << std::endl;
        m_scheduler.schedule(ndn::time::milliseconds(0), takeoff);
        m_scheduler.schedule(ndn::time::milliseconds(5000), orbit);
    };

    auto OutputMetrics = [&]() {
        takeoff_metric.printStats();
        takeoff_metric.exportCSV("wuas_takeoff.csv");
        orbit_metric.printStats();
        orbit_metric.exportCSV("wuas_orbit.csv");
    };

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

    NDN_LOG_INFO("WUAS running");
    try {
        while (1) {
            if (m_telemetry.in_air() && !single_request_sent) {
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