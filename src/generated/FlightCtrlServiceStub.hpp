#ifndef FlightCtrlServiceStub_HPP
#define FlightCtrlServiceStub_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>


#include "ndn-service-framework/ServiceUser.hpp"
#include "messages.pb.h"

#include <iostream>
#include <string>
#include <regex>

namespace muas
{
    
    using SwitchMode_Callback = std::function<void(const muas::FlightCtrl_SwitchMode_Response &)>;
    
    using Takeoff_Callback = std::function<void(const muas::FlightCtrl_Takeoff_Response &)>;
    
    using Land_Callback = std::function<void(const muas::FlightCtrl_Land_Response &)>;
    
    using RTL_Callback = std::function<void(const muas::FlightCtrl_RTL_Response &)>;
    
    using Kill_Callback = std::function<void(const muas::FlightCtrl_Kill_Response &)>;
    
    using SetSpeed_Callback = std::function<void(const muas::FlightCtrl_SetSpeed_Response &)>;
    
    using Reposition_Callback = std::function<void(const muas::FlightCtrl_Reposition_Response &)>;
    

    class FlightCtrlServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        FlightCtrlServiceStub(ndn_service_framework::ServiceUser& user);
        virtual ~FlightCtrlServiceStub();

        
        void SwitchMode_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_SwitchMode_Request &_request, muas::SwitchMode_Callback _callback,  const size_t strategy);
        
        void Takeoff_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Takeoff_Request &_request, muas::Takeoff_Callback _callback,  const size_t strategy);
        
        void Land_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Land_Request &_request, muas::Land_Callback _callback,  const size_t strategy);
        
        void RTL_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_RTL_Request &_request, muas::RTL_Callback _callback,  const size_t strategy);
        
        void Kill_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Kill_Request &_request, muas::Kill_Callback _callback,  const size_t strategy);
        
        void SetSpeed_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_SetSpeed_Request &_request, muas::SetSpeed_Callback _callback,  const size_t strategy);
        
        void Reposition_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Reposition_Request &_request, muas::Reposition_Callback _callback,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,SwitchMode_Callback> SwitchMode_Callbacks;
        
        std::map<ndn::Name,Takeoff_Callback> Takeoff_Callbacks;
        
        std::map<ndn::Name,Land_Callback> Land_Callbacks;
        
        std::map<ndn::Name,RTL_Callback> RTL_Callbacks;
        
        std::map<ndn::Name,Kill_Callback> Kill_Callbacks;
        
        std::map<ndn::Name,SetSpeed_Callback> SetSpeed_Callbacks;
        
        std::map<ndn::Name,Reposition_Callback> Reposition_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif