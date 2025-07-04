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
    
    using CaptureSingle_Callback = std::function<void(const muas::SensorCtrl_CaptureSingle_Response &)>;
    
    using CapturePeriodic_Callback = std::function<void(const muas::SensorCtrl_CapturePeriodic_Response &)>;
    

    class SensorServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        SensorServiceStub(ndn_service_framework::ServiceUser& user);
        virtual ~SensorServiceStub();

        
        void GetSensorInfo_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_GetSensorInfo_Request &_request, muas::GetSensorInfo_Callback _callback,  const size_t strategy);
        
        void CaptureSingle_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_CaptureSingle_Request &_request, muas::CaptureSingle_Callback _callback,  const size_t strategy);
        
        void CapturePeriodic_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_CapturePeriodic_Request &_request, muas::CapturePeriodic_Callback _callback,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,GetSensorInfo_Callback> GetSensorInfo_Callbacks;
        
        std::map<ndn::Name,CaptureSingle_Callback> CaptureSingle_Callbacks;
        
        std::map<ndn::Name,CapturePeriodic_Callback> CapturePeriodic_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif