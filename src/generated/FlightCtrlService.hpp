#ifndef FlightCtrlService_HPP
#define FlightCtrlService_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>

#include "ndn-service-framework/ServiceProvider.hpp"
#include "ndn-service-framework/ServiceStub.hpp"
#include "ndn-service-framework/Service.hpp"

#include "messages.pb.h"

#include <vector>

namespace muas
{

    
    using SwitchMode_Function = std::function<void(const ndn::Name &, const muas::FlightCtrl_SwitchMode_Request &, muas::FlightCtrl_SwitchMode_Response &)>;
    
    using Takeoff_Function = std::function<void(const ndn::Name &, const muas::FlightCtrl_Takeoff_Request &, muas::FlightCtrl_Takeoff_Response &)>;
    
    using Land_Function = std::function<void(const ndn::Name &, const muas::FlightCtrl_Land_Request &, muas::FlightCtrl_Land_Response &)>;
    
    using RTL_Function = std::function<void(const ndn::Name &, const muas::FlightCtrl_RTL_Request &, muas::FlightCtrl_RTL_Response &)>;
    
    using Kill_Function = std::function<void(const ndn::Name &, const muas::FlightCtrl_Kill_Request &, muas::FlightCtrl_Kill_Response &)>;
    
    using SetSpeed_Function = std::function<void(const ndn::Name &, const muas::FlightCtrl_SetSpeed_Request &, muas::FlightCtrl_SetSpeed_Response &)>;
    
    using Reposition_Function = std::function<void(const ndn::Name &, const muas::FlightCtrl_Reposition_Request &, muas::FlightCtrl_Reposition_Response &)>;
    

    class FlightCtrlService : public ndn_service_framework::Service
    {
    public:
        FlightCtrlService(ndn_service_framework::ServiceProvider &serviceProvider)
            : ndn_service_framework::Service(serviceProvider),
              serviceName("FlightCtrl")
        {
        }

        virtual ~FlightCtrlService();

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;
        
        void SwitchMode(const ndn::Name &requesterIdentity, const muas::FlightCtrl_SwitchMode_Request &_request, muas::FlightCtrl_SwitchMode_Response &_response);
        
        void Takeoff(const ndn::Name &requesterIdentity, const muas::FlightCtrl_Takeoff_Request &_request, muas::FlightCtrl_Takeoff_Response &_response);
        
        void Land(const ndn::Name &requesterIdentity, const muas::FlightCtrl_Land_Request &_request, muas::FlightCtrl_Land_Response &_response);
        
        void RTL(const ndn::Name &requesterIdentity, const muas::FlightCtrl_RTL_Request &_request, muas::FlightCtrl_RTL_Response &_response);
        
        void Kill(const ndn::Name &requesterIdentity, const muas::FlightCtrl_Kill_Request &_request, muas::FlightCtrl_Kill_Response &_response);
        
        void SetSpeed(const ndn::Name &requesterIdentity, const muas::FlightCtrl_SetSpeed_Request &_request, muas::FlightCtrl_SetSpeed_Response &_response);
        
        void Reposition(const ndn::Name &requesterIdentity, const muas::FlightCtrl_Reposition_Request &_request, muas::FlightCtrl_Reposition_Response &_response);
        


    public:
        ndn::Name serviceName;
        
        SwitchMode_Function SwitchMode_Handler;
        
        Takeoff_Function Takeoff_Handler;
        
        Land_Function Land_Handler;
        
        RTL_Function RTL_Handler;
        
        Kill_Function Kill_Handler;
        
        SetSpeed_Function SetSpeed_Handler;
        
        Reposition_Function Reposition_Handler;
        
    };
}
#endif