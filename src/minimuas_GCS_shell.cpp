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
    nameStream << producer_id << "/sensor/" << sensor_id << "/" << idx << ".png";
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

int main(int argc, char **argv)
{
    Metrics takeoff_metric(true, true);
    Metrics getinfo_metric(true, true);
    Metrics capture_metric(true, true);
    Metrics ping_metric(true, true);

    if (argc != 2) {
        std::cerr << "Usage: gcs-shell <identity>" << std::endl;
        return 1;
    }

    std::string identity = argv[1];
    std::string conf_dir = "/usr/local/bin";
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

    muas::ServiceUser_GCS m_serviceUser(
        m_face, "/muas",
        gs_certificate,
        m_keyChain.getPib().getIdentity("/muas/aa").getDefaultKey().getDefaultCertificate(),
        conf_dir + "/trust-any.conf"
    );

    std::vector<ndn::Name> wuas_providers = { ndn::Name("/muas/wuas-01") };
    std::vector<ndn::Name> iuas_providers = { ndn::Name("/muas/iuas-01") };
    std::vector<ndn::Name> uas_providers = { ndn::Name("/muas/iuas-01"), ndn::Name("/muas/wuas-01") };

    m_face.processEvents(ndn::time::milliseconds(10000));

    auto wuas_takeoff_call = [&]() {
        auto takeoff_start = takeoff_metric.start();
        std::cout << "Requesting takeoff from WUAS." << std::endl;
        muas::FlightCtrl_Takeoff_Request takeoff_request;
        m_serviceUser.Takeoff_Async(wuas_providers, takeoff_request,
            [&, takeoff_start](const muas::FlightCtrl_Takeoff_Response& _response) {
                takeoff_metric.end(takeoff_start, true);
                NDN_LOG_INFO(_response.DebugString());
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto iuas_takeoff_call = [&]() {
        auto takeoff_start = takeoff_metric.start();
        std::cout << "Requesting takeoff from IUAS." << std::endl;
        muas::FlightCtrl_Takeoff_Request takeoff_request;
        m_serviceUser.Takeoff_Async(iuas_providers, takeoff_request,
            [&, takeoff_start](const muas::FlightCtrl_Takeoff_Response& _response) {
                takeoff_metric.end(takeoff_start, true);
                NDN_LOG_INFO(_response.DebugString());
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto rtl_call = [&](std::vector<ndn::Name> uas_providers) {
        std::cout << "Requesting RTL." << std::endl;
        muas::FlightCtrl_RTL_Request rtl_request;
        m_serviceUser.RTL_Async(uas_providers, rtl_request,
            [&](const muas::FlightCtrl_RTL_Response& _response) {
                NDN_LOG_INFO(_response.DebugString());
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto kill_call = [&](std::vector<ndn::Name> uas_providers) {
        std::cout << "Requesting Kill." << std::endl;
        muas::FlightCtrl_Kill_Request kill_request;
        m_serviceUser.Kill_Async(uas_providers, kill_request,
            [&](const muas::FlightCtrl_Kill_Response& _response) {
                NDN_LOG_INFO(_response.DebugString());
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto info_call = [&]() {
        auto getinfo_start = getinfo_metric.start();
        std::cout << "Requesting sensor info from IUAS." << std::endl;
        muas::SensorCtrl_GetSensorInfo_Request sensor_info_request;
        m_serviceUser.GetSensorInfo_Async(iuas_providers, sensor_info_request,
            [&, getinfo_start](const muas::SensorCtrl_GetSensorInfo_Response& _response) {
                getinfo_metric.end(getinfo_start, true);
                if (_response.sensors_size() > 0) {
                    iuas_sensor_idx = _response.sensors(0).id();
                    NDN_LOG_INFO(_response.DebugString());
                } else {
                    std::cerr << "No sensors found." << std::endl;
                }
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto cap_call = [&](int idx) {
        auto capture_start = capture_metric.start();
        std::cout << "Requesting sensor capture from IUAS." << std::endl;
        muas::SensorCtrl_CaptureSingle_Request sensor_cap_request;
        m_serviceUser.CaptureSingle_Async(iuas_providers, sensor_cap_request, [&, capture_start, idx](const muas::SensorCtrl_CaptureSingle_Response& _response) {
                capture_metric.end(capture_start, true);
                NDN_LOG_INFO(_response.DebugString());
                int img_idx = std::stoi(_response.capture_id());
                std::thread([=]() {
                    getCapture(iuas_providers.at(0).toUri(), iuas_sensor_idx, img_idx);
                }).detach();
            },
            ndn_service_framework::tlv::NoCoordination
        );
    };

    auto echo_call = [&]() {
        auto ping_start = ping_metric.start();
        std::cout << "Requesting ping from some UAS." << std::endl;
        muas::Entity_Echo_Request echo_request;
        echo_request.set_nonce("test");
        m_serviceUser.Echo_Async(uas_providers, echo_request, [&, ping_start](const muas::Entity_Echo_Response& _response) {
                ping_metric.end(ping_start, true);
                NDN_LOG_INFO(_response.DebugString());
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
        ping_metric.printStats();
        ping_metric.exportCSV("gcs_ping.csv");
    };

    auto ClearMetrics = [&]() {
        takeoff_metric.reset();
        getinfo_metric.reset();
        capture_metric.reset();
        ping_metric.reset();
    };

    std::atomic<bool> running(true);

    // Start NDN event processing in a background thread
    std::thread faceThread([&]() {
        try {
            m_face.processEvents(ndn::time::milliseconds(0), true);
        } catch (const std::exception& e) {
            std::cerr << "Face error: " << e.what() << std::endl;
        }
    });

    // ===== Interactive Command Loop =====
    std::string input;
    std::cout << "GCS Interactive CLI started. Type `help` for commands." << std::endl;

    while (running) {
        std::cout << "> ";
        std::getline(std::cin, input);
        std::istringstream iss(input);
        std::string command;
        iss >> command;

        if (command == "exit" || command == "quit") {
            running = false;
            m_face.getIoContext().stop();
        } else if (command == "ping") {
            int interval, count;
            if (iss >> interval >> count) {
                for (int i = 0; i < count; ++i) {
                    m_scheduler.schedule(ndn::time::milliseconds(i * interval), [&, i] { echo_call(); });
                }
            } else {
                std::cerr << "Usage: ping <interval_ms> <count>" << std::endl;
            }
        } else if (command == "rtl") {
            std::string uas;
            if (iss >> uas) {
                std::stringstream nameStream;
                nameStream << "/muas/" << uas << "-01";
                std::string name = nameStream.str();
                std::vector<ndn::Name> uas_providers = { ndn::Name(name) };
                m_scheduler.schedule(ndn::time::milliseconds(0), [&, uas_providers] { rtl_call(uas_providers); });
            } else {
                std::cerr << "Usage: rtl <uas> (wuas/iuas)" << std::endl;
            }
        }  else if (command == "kill") {
            std::string uas;
            if (iss >> uas) {
                std::stringstream nameStream;
                nameStream << "/muas/" << uas << "-01";
                std::string name = nameStream.str();
                std::vector<ndn::Name> uas_providers = { ndn::Name(name) };
                m_scheduler.schedule(ndn::time::milliseconds(0), [&, uas_providers] { kill_call(uas_providers); });
            } else {
                std::cerr << "Usage: kill <uas> (wuas/iuas)" << std::endl;
            }
        } else if (command == "iuas_takeoff") {
            m_scheduler.schedule(ndn::time::milliseconds(0), iuas_takeoff_call);
        } else if (command == "wuas_takeoff") {
            m_scheduler.schedule(ndn::time::milliseconds(0), wuas_takeoff_call);
        } else if (command == "get_info") {
            m_scheduler.schedule(ndn::time::milliseconds(0), info_call);
        } else if (command == "metrics") {
            OutputMetrics();
        } else if (command == "clear") {
            ClearMetrics();
        } else if (command == "capture") {
            int interval, count;
            if (iss >> interval >> count) {
                for (int i = 0; i < count; ++i) {
                    m_scheduler.schedule(ndn::time::milliseconds(i * interval), [&, i] { cap_call(i); });
                }
            } else {
                std::cerr << "Usage: capture <interval_ms> <count>" << std::endl;
            }
        } else if (command == "help") {
            std::cout << "Available commands:\n"
                      << "  ping <i> <n>    - Ping n times with i ms interval\n"
                      << "  iuas_takeoff    - Request takeoff from IUAS\n"
                      << "  wuas_takeoff    - Request takeoff from WUAS\n"
                      << "  rtl <uas>       - Request RTL from UAS\n"
                      << "  kill <uas>      - Request kill from UAS\n"
                      << "  get_info        - Request sensor info from IUAS\n"
                      << "  capture <i> <n> - Capture n images with i ms interval\n"
                      << "  metrics         - Output metrics\n"
                      << "  clear           - Clear metrics\n"
                      << "  exit/quit       - Exit the program\n";
        } else {
            std::cerr << "Unknown command: " << command << std::endl;
        }
    }

    faceThread.join();  // Wait for the Face thread to finish
    return 0;
}