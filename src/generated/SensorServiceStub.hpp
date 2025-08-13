#ifndef SensorServiceStub_HPP
#define SensorServiceStub_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>


#include "ndn-service-framework/ServiceUser.hpp"
#include "messages.pb.h"

#include <iostream>
#include <string>
#include <regex>

namespace muas
{
    
    using GetSensorInfo_Callback = std::function<void(const muas::SensorCtrl_GetSensorInfo_Response &)>;
    using GetSensorInfo_Timeout_Callback = std::function<void(const muas::SensorCtrl_GetSensorInfo_Request &)>;
    
    using CaptureSingle_Callback = std::function<void(const muas::SensorCtrl_CaptureSingle_Response &)>;
    using CaptureSingle_Timeout_Callback = std::function<void(const muas::SensorCtrl_CaptureSingle_Request &)>;
    
    using CapturePeriodic_Callback = std::function<void(const muas::SensorCtrl_CapturePeriodic_Response &)>;
    using CapturePeriodic_Timeout_Callback = std::function<void(const muas::SensorCtrl_CapturePeriodic_Request &)>;
    

    class SensorServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        SensorServiceStub(ndn::Face& face, ndn_service_framework::ServiceUser& user);
        virtual ~SensorServiceStub();

        
        void GetSensorInfo_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_GetSensorInfo_Request &_request, muas::GetSensorInfo_Callback _callback, muas::GetSensorInfo_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void CaptureSingle_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_CaptureSingle_Request &_request, muas::CaptureSingle_Callback _callback, muas::CaptureSingle_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void CapturePeriodic_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_CapturePeriodic_Request &_request, muas::CapturePeriodic_Callback _callback, muas::CapturePeriodic_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,GetSensorInfo_Callback> GetSensorInfo_Callbacks;
        std::map<ndn::Name,GetSensorInfo_Timeout_Callback> GetSensorInfo_Timeout_Callbacks;
        
        std::map<ndn::Name,CaptureSingle_Callback> CaptureSingle_Callbacks;
        std::map<ndn::Name,CaptureSingle_Timeout_Callback> CaptureSingle_Timeout_Callbacks;
        
        std::map<ndn::Name,CapturePeriodic_Callback> CapturePeriodic_Callbacks;
        std::map<ndn::Name,CapturePeriodic_Timeout_Callback> CapturePeriodic_Timeout_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif