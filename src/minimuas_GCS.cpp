#include <boost/process.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/time.h>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>
#include "./generated/ServiceUser_GCS.hpp"

#include "./metrics.hpp"

NDN_LOG_INIT(muas.main_gcs);

void getCapture(const std::string& producer_id, int sensor_id, int idx) {
    namespace bp = boost::process;

    std::stringstream nameStream, filenameStream;
    nameStream << producer_id << "/sensor/" << sensor_id << "/" << idx;
    filenameStream << idx << ".png";

    std::string name = nameStream.str();
    std::string filename = filenameStream.str();

    try {
        NDN_LOG_INFO("ndnget " + name + " > " + filename);

        // Pass the filename directly as an output redirection target
        bp::child c("ndnget", name, bp::std_out > filename);
        c.wait();

        std::cout << "Saved " << filename << std::endl;
    } catch (const bp::process_error& e) {
        std::cerr << "Failed to run ndnget: " << e.what() << std::endl;
    }
}

int
main(int argc, char **argv)
{
    Metrics takeoff_metric(true, true);
    Metrics getinfo_metric(true, true);
    Metrics capture_metric(true, true);

    if (argc != 4)
    {
        std::cerr << "Usage: gcs <identity> <capture_interval_in_ms> <count>" << std::endl;
        exit(1);
    }
    std::string identity = argv[1];
    int interval_in_ms = std::stoi(argv[2]);
    int count = std::stoi(argv[3]);
    std::string conf_dir = "/usr/local/bin";
    int delay = 0000;

    int iuas_sensor_idx = 0;

    ndn::Face m_face;
    ndn::Scheduler m_scheduler(m_face.getIoContext());
    ndn::security::KeyChain m_keyChain;
    ndn::security::Certificate gs_certificate(
        m_keyChain
            .getPib()
            .getIdentity(identity)
            .getDefaultKey()
            .getDefaultCertificate()
    );

    muas::ServiceUser_GCS m_serviceUser(m_face, "/muas",gs_certificate,m_keyChain.getPib().getIdentity("/muas/aa").getDefaultKey().getDefaultCertificate(), conf_dir + "/trust-any.conf");
    
    std::vector<ndn::Name> wuas_providers;
    wuas_providers.push_back(ndn::Name("/muas/wuas-01"));
    std::vector<ndn::Name> iuas_providers;
    iuas_providers.push_back(ndn::Name("/muas/iuas-01"));

    m_face.processEvents(ndn::time::milliseconds(2000));

    auto wuas_takeoff_call = [&]() {
        struct timeval tv;
        auto takeoff_start = takeoff_metric.start();
        std::cout << "Requesting takeoff from WUAS." << std::endl;
        muas::FlightCtrl_Takeoff_Request takeoff_request;

        gettimeofday(&tv, NULL);
        google::protobuf::Timestamp time_req_sent;
        time_req_sent.set_seconds(tv.tv_sec);
        time_req_sent.set_nanos(tv.tv_usec * 1000);
        takeoff_request.set_allocated_time_request_sent(&time_req_sent);

        m_serviceUser.Takeoff_Async(wuas_providers, takeoff_request,
            [&, takeoff_start](const muas::FlightCtrl_Takeoff_Response& _response) {
                takeoff_metric.end(takeoff_start, true);
                NDN_LOG_INFO(_response.DebugString());

                struct timeval tv;
                gettimeofday(&tv, NULL);

                google::protobuf::Timestamp time_res_recv;
                time_res_recv.set_seconds(tv.tv_sec);
                time_res_recv.set_nanos(tv.tv_usec * 1000);
                auto time_req_recv = _response.time_request_received();
                auto time_res_sent = _response.time_response_sent();

                auto req_latency_sec = time_req_recv.seconds() - time_req_sent.seconds();
                auto req_latency_nanos = time_req_recv.nanos() - time_req_sent.nanos();
                auto req_latency_ms = req_latency_sec*1000 + (req_latency_nanos/100000);

                auto res_latency_sec = time_res_recv.seconds() - time_res_sent.seconds();
                auto res_latency_nanos = time_res_recv.nanos() - time_res_sent.nanos();
                auto res_latency_ms = res_latency_sec*1000 + (res_latency_nanos/100000);

                NDN_LOG_INFO("Request latency: " << req_latency_ms << " ms / Response latency: " << res_latency_ms << " ms");
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto iuas_takeoff_call = [&]() {
        struct timeval tv;
        auto takeoff_start = takeoff_metric.start();
        std::cout << "Requesting takeoff from IUAS." << std::endl;
        muas::FlightCtrl_Takeoff_Request takeoff_request;

        gettimeofday(&tv, NULL);
        google::protobuf::Timestamp time_req_sent;
        time_req_sent.set_seconds(tv.tv_sec);
        time_req_sent.set_nanos(tv.tv_usec * 1000);
        takeoff_request.set_allocated_time_request_sent(&time_req_sent);

        m_serviceUser.Takeoff_Async(iuas_providers, takeoff_request,
            [&, takeoff_start](const muas::FlightCtrl_Takeoff_Response& _response) {
                takeoff_metric.end(takeoff_start, true);
                NDN_LOG_INFO(_response.DebugString());

                struct timeval tv;
                gettimeofday(&tv, NULL);

                google::protobuf::Timestamp time_res_recv;
                time_res_recv.set_seconds(tv.tv_sec);
                time_res_recv.set_nanos(tv.tv_usec * 1000);
                auto time_req_recv = _response.time_request_received();
                auto time_res_sent = _response.time_response_sent();

                auto req_latency_sec = time_req_recv.seconds() - time_req_sent.seconds();
                auto req_latency_nanos = time_req_recv.nanos() - time_req_sent.nanos();
                auto req_latency_ms = req_latency_sec*1000 + (req_latency_nanos/100000);

                auto res_latency_sec = time_res_recv.seconds() - time_res_sent.seconds();
                auto res_latency_nanos = time_res_recv.nanos() - time_res_sent.nanos();
                auto res_latency_ms = res_latency_sec*1000 + (res_latency_nanos/100000);

                NDN_LOG_INFO("Request latency: " << req_latency_ms << " ms / Response latency: " << res_latency_ms << " ms");
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto info_call = [&]() {
        struct timeval tv;
        auto getinfo_start = getinfo_metric.start();
        std::cout << "Requesting sensor info from IUAS." << std::endl;
        muas::SensorCtrl_GetSensorInfo_Request sensor_info_request;

        gettimeofday(&tv, NULL);
        google::protobuf::Timestamp time_req_sent;
        time_req_sent.set_seconds(tv.tv_sec);
        time_req_sent.set_nanos(tv.tv_usec * 1000);
        sensor_info_request.set_allocated_time_request_sent(&time_req_sent);

        m_serviceUser.GetSensorInfo_Async(iuas_providers, sensor_info_request,
            [&, getinfo_start](const muas::SensorCtrl_GetSensorInfo_Response& _response) {
                getinfo_metric.end(getinfo_start, true);
                if (_response.sensors_size() > 0) {
                    iuas_sensor_idx = _response.sensors(0).id();
                    NDN_LOG_INFO(_response.DebugString());
                } else {
                    std::cerr << "No sensors found." << std::endl;
                }

                struct timeval tv;
                gettimeofday(&tv, NULL);

                google::protobuf::Timestamp time_res_recv;
                time_res_recv.set_seconds(tv.tv_sec);
                time_res_recv.set_nanos(tv.tv_usec * 1000);
                auto time_req_recv = _response.time_request_received();
                auto time_res_sent = _response.time_response_sent();

                auto req_latency_sec = time_req_recv.seconds() - time_req_sent.seconds();
                auto req_latency_nanos = time_req_recv.nanos() - time_req_sent.nanos();
                auto req_latency_ms = req_latency_sec*1000 + (req_latency_nanos/100000);

                auto res_latency_sec = time_res_recv.seconds() - time_res_sent.seconds();
                auto res_latency_nanos = time_res_recv.nanos() - time_res_sent.nanos();
                auto res_latency_ms = res_latency_sec*1000 + (res_latency_nanos/100000);

                NDN_LOG_INFO("Request latency: " << req_latency_ms << " ms / Response latency: " << res_latency_ms << " ms");
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto cap_call = [&]() {
        struct timeval tv;
        auto capture_start = capture_metric.start();
        std::cout << "Requesting sensor capture from IUAS." << std::endl;
        muas::SensorCtrl_CaptureSingle_Request sensor_cap_request;

        gettimeofday(&tv, NULL);
        google::protobuf::Timestamp time_req_sent;
        time_req_sent.set_seconds(tv.tv_sec);
        time_req_sent.set_nanos(tv.tv_usec * 1000);
        sensor_cap_request.set_allocated_time_request_sent(&time_req_sent);

        m_serviceUser.CaptureSingle_Async(iuas_providers, sensor_cap_request, [&, capture_start](const muas::SensorCtrl_CaptureSingle_Response& _response) {
                capture_metric.end(capture_start, true);
                NDN_LOG_INFO(_response.DebugString());

                struct timeval tv;
                gettimeofday(&tv, NULL);

                google::protobuf::Timestamp time_res_recv;
                time_res_recv.set_seconds(tv.tv_sec);
                time_res_recv.set_nanos(tv.tv_usec * 1000);
                auto time_req_recv = _response.time_request_received();
                auto time_res_sent = _response.time_response_sent();

                auto req_latency_sec = time_req_recv.seconds() - time_req_sent.seconds();
                auto req_latency_nanos = time_req_recv.nanos() - time_req_sent.nanos();
                auto req_latency_ms = req_latency_sec*1000 + (req_latency_nanos/100000);

                auto res_latency_sec = time_res_recv.seconds() - time_res_sent.seconds();
                auto res_latency_nanos = time_res_recv.nanos() - time_res_sent.nanos();
                auto res_latency_ms = res_latency_sec*1000 + (res_latency_nanos/100000);

                NDN_LOG_INFO("Request latency: " << req_latency_ms << " ms / Response latency: " << res_latency_ms << " ms");

                int img_idx = std::stoi(_response.capture_id());
                std::thread([=]() {
                    getCapture(iuas_providers.at(0).toUri(), iuas_sensor_idx, img_idx);
                }).detach();
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto OutputMetrics = [&]() {
        takeoff_metric.printStats();
        takeoff_metric.exportCSV("gcs_takeoff.csv");
        getinfo_metric.printStats();
        getinfo_metric.exportCSV("gcs_getinfo.csv");
        capture_metric.printStats();
        capture_metric.exportCSV("gcs_capture.csv");
    };  

    for (int i = 0; i < count; i++)
    {
        m_scheduler.schedule(ndn::time::milliseconds(delay+interval_in_ms*i), cap_call);
    }

    m_scheduler.schedule(ndn::time::milliseconds(10000), info_call);
    m_scheduler.schedule(ndn::time::milliseconds(10000), wuas_takeoff_call);
    m_scheduler.schedule(ndn::time::milliseconds(20000), iuas_takeoff_call);
    m_scheduler.schedule(ndn::time::milliseconds(30000), OutputMetrics);

    NDN_LOG_INFO("GCS running");
    try {
        m_face.processEvents(ndn::time::milliseconds(0),true);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    
}