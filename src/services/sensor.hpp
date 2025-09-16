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

#include "../util/latency.hpp"

using std::chrono::seconds;
using std::this_thread::sleep_for;

/// TODO: rewrite sensor capture implementation
int get_next_file_number(const char *directory) {
    DIR *dir;
    struct dirent *entry;
    int max_num = -1;

    dir = opendir(directory);
    if (dir == NULL) {
        perror("opendir");
        return 0;
    }

    // Has something to do with the numerical file naming
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

/// Request service provider's sensor payload information
auto getSensorInfo(muas::Sensor sensor) {
    auto getSensorInfoHandler = [&, sensor](const ndn::Name& requesterIdentity, const muas::SensorCtrl_GetSensorInfo_Request& _request, muas::SensorCtrl_GetSensorInfo_Response& _response){
        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = set_request_ts(time_req_sent);
        
        NDN_LOG_INFO("SensorInfo request received");
        NDN_LOG_INFO("SensorInfo request latency: " << req_latency_ms << " ms");

        // Create a sensor object and adding sensor attributes
        muas::Sensor* s = _response.add_sensors();
        s->set_name(sensor.name());
        s->set_id(sensor.id());
        s->set_type(sensor.type());
        s->set_data_namespace(sensor.data_namespace());

        _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
        _response.mutable_response()->set_msg("Sensor info request satisfied.");
        set_response_ts(time_req_recv, _response);
    };

    return getSensorInfoHandler;
}

/// Request service provider to capture a single reading from a sensor
auto captureSingle() {
    auto captureSingleHandler = [&](const ndn::Name& requesterIdentity, const muas::SensorCtrl_CaptureSingle_Request& _request, muas::SensorCtrl_CaptureSingle_Response& _response){
        auto time_req_sent = _request.time_request_sent();
        auto [req_latency_ms, time_req_recv] = set_request_ts(time_req_sent);
        
        NDN_LOG_INFO("CaptureSingle request received");
        NDN_LOG_INFO("CaptureSingle request latency: " << req_latency_ms << " ms");

        // Open the video device which is hopefully at index 0
        int cam_idx = 0;
        std::string cap_dev = "v4l2:///dev/video";
        std::string cap_str = cap_dev + std::to_string(cam_idx);
        std::cout << "Trying to open camera (" << cap_str << ")..." << std::endl;
        
        // Create a video capture object
        cv::VideoCapture capture(cap_str, cv::CAP_V4L2);
        if (!capture.isOpened())
        {
            NDN_LOG_ERROR("ERROR: Can't initialize camera (" << cam_idx << ")");

            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_ERROR);
            _response.mutable_response()->set_msg("Camera failed to initialize");
            _response.set_capture_id(std::to_string(-1));
            set_response_ts(time_req_recv, _response);
        }
        else
        {
            cv::Mat frame; // create a frame

            capture >> frame;
            const char *directory = "./captures";  // in program directory
            int next_num = get_next_file_number(directory);

            // Naming the file based on current ones
            char filename[256];
            snprintf(filename, sizeof(filename), "%s/%d.png", directory, next_num);
            
            // Write frame to file
            cv::imwrite(filename,frame);
            NDN_LOG_INFO("Saved " << filename);

            char msg[200];

            snprintf(msg,sizeof(msg),"Single capture successful. Index: %i", next_num);

            _response.mutable_response()->set_code(muas::NDNSF_Response_miniMUAS_Code_SUCCESS);
            _response.mutable_response()->set_msg(msg);
            _response.set_capture_id(std::to_string(next_num));
            set_response_ts(time_req_recv, _response);        }
    };

    return captureSingleHandler;
}