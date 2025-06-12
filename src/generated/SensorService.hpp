#ifndef SensorService_HPP
#define SensorService_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>

#include "ndn-service-framework/ServiceProvider.hpp"
#include "ndn-service-framework/ServiceStub.hpp"
#include "ndn-service-framework/Service.hpp"

#include "messages.pb.h"

#include <vector>

namespace muas
{

    
    using GetSensorInfo_Function = std::function<void(const ndn::Name &, const muas::SensorCtrl_GetSensorInfo_Request &, muas::SensorCtrl_GetSensorInfo_Response &)>;
    
    using CaptureSingle_Function = std::function<void(const ndn::Name &, const muas::SensorCtrl_CaptureSingle_Request &, muas::SensorCtrl_CaptureSingle_Response &)>;
    
    using CapturePeriodic_Function = std::function<void(const ndn::Name &, const muas::SensorCtrl_CapturePeriodic_Request &, muas::SensorCtrl_CapturePeriodic_Response &)>;
    

    class SensorService : public ndn_service_framework::Service
    {
    public:
        SensorService(ndn_service_framework::ServiceProvider &serviceProvider)
            : ndn_service_framework::Service(serviceProvider),
              serviceName("Sensor")
        {
        }

        virtual ~SensorService();

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;
        
        void GetSensorInfo(const ndn::Name &requesterIdentity, const muas::SensorCtrl_GetSensorInfo_Request &_request, muas::SensorCtrl_GetSensorInfo_Response &_response);
        
        void CaptureSingle(const ndn::Name &requesterIdentity, const muas::SensorCtrl_CaptureSingle_Request &_request, muas::SensorCtrl_CaptureSingle_Response &_response);
        
        void CapturePeriodic(const ndn::Name &requesterIdentity, const muas::SensorCtrl_CapturePeriodic_Request &_request, muas::SensorCtrl_CapturePeriodic_Response &_response);
        


    public:
        ndn::Name serviceName;
        
        GetSensorInfo_Function GetSensorInfo_Handler;
        
        CaptureSingle_Function CaptureSingle_Handler;
        
        CapturePeriodic_Function CapturePeriodic_Handler;
        
    };
}
#endif