#include <boost/process.hpp>
#include <iostream>
#include <sstream>
#include <string>

#include "./generated/messages.pb.h"
#include <ndn-service-framework/common.hpp>
#include "./generated/ServiceUser_GCS.hpp"

#include <mutex>
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
        auto takeoff_start = takeoff_metric.start();
        std::cout << "Requesting takeoff from WUAS." << std::endl;
        muas::FlightCtrl_Takeoff_Request takeoff_request;
        m_serviceUser.Takeoff_Async(wuas_providers, takeoff_request, [&, takeoff_start](const muas::FlightCtrl_Takeoff_Response& _response){
                takeoff_metric.end(takeoff_start, true);
                NDN_LOG_INFO(_response.DebugString());
            }
            , ndn_service_framework::tlv::NoCoordination
        );
    };

    auto iuas_takeoff_call = [&]() {
        auto takeoff_start = takeoff_metric.start();
        std::cout << "Requesting takeoff from WUAS." << std::endl;
        muas::FlightCtrl_Takeoff_Request takeoff_request;
        m_serviceUser.Takeoff_Async(iuas_providers, takeoff_request, [&, takeoff_start](const muas::FlightCtrl_Takeoff_Response& _response){
                takeoff_metric.end(takeoff_start, true);
                NDN_LOG_INFO(_response.DebugString());
            }
            , ndn_service_framework::tlv::NoCoordination
        );
    };

    auto info_call = [&]() {
        auto getinfo_start = getinfo_metric.start();
        std::cout << "Requesting sensor info from IUAS." << std::endl;
        muas::SensorCtrl_GetSensorInfo_Request sensor_info_request;
        m_serviceUser.GetSensorInfo_Async(iuas_providers, sensor_info_request, [&, getinfo_start](const muas::SensorCtrl_GetSensorInfo_Response& _response){
                getinfo_metric.end(getinfo_start, true);
                muas::Sensor s = _response.sensors(0);
                NDN_LOG_INFO(_response.DebugString());
                iuas_sensor_idx = s.id();
                NDN_LOG_INFO(iuas_sensor_idx);
            }
            , ndn_service_framework::tlv::NoCoordination
        );
    };

    auto cap_call = [&](){
        auto capture_start = capture_metric.start();
        std::cout << "Requesting sensor capture from IUAS." << std::endl;
        muas::SensorCtrl_CaptureSingle_Request sensor_cap_request;
        m_serviceUser.CaptureSingle_Async(iuas_providers, sensor_cap_request, [&, capture_start](const muas::SensorCtrl_CaptureSingle_Response& _response) {
                capture_metric.end(capture_start, true);
                NDN_LOG_INFO(_response.DebugString());
                int idx = std::stoi(_response.capture_id());

                // Run in background thread
                std::thread([=]() {
                    getCapture(iuas_providers.at(0).toUri(), iuas_sensor_idx, idx);
                }).detach();  // Detached thread so it runs independently

                // TODO: Next action in test via space key
            }
            , ndn_service_framework::tlv::NoCoordination
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