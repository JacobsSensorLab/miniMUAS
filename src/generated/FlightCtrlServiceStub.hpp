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
    using SwitchMode_Timeout_Callback = std::function<void(const muas::FlightCtrl_SwitchMode_Request &)>;
    
    using Takeoff_Callback = std::function<void(const muas::FlightCtrl_Takeoff_Response &)>;
    using Takeoff_Timeout_Callback = std::function<void(const muas::FlightCtrl_Takeoff_Request &)>;
    
    using Land_Callback = std::function<void(const muas::FlightCtrl_Land_Response &)>;
    using Land_Timeout_Callback = std::function<void(const muas::FlightCtrl_Land_Request &)>;
    
    using RTL_Callback = std::function<void(const muas::FlightCtrl_RTL_Response &)>;
    using RTL_Timeout_Callback = std::function<void(const muas::FlightCtrl_RTL_Request &)>;
    
    using Kill_Callback = std::function<void(const muas::FlightCtrl_Kill_Response &)>;
    using Kill_Timeout_Callback = std::function<void(const muas::FlightCtrl_Kill_Request &)>;
    
    using SetSpeed_Callback = std::function<void(const muas::FlightCtrl_SetSpeed_Response &)>;
    using SetSpeed_Timeout_Callback = std::function<void(const muas::FlightCtrl_SetSpeed_Request &)>;
    
    using Reposition_Callback = std::function<void(const muas::FlightCtrl_Reposition_Response &)>;
    using Reposition_Timeout_Callback = std::function<void(const muas::FlightCtrl_Reposition_Request &)>;
    

    class FlightCtrlServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        FlightCtrlServiceStub(ndn_service_framework::ServiceUser& user);
        virtual ~FlightCtrlServiceStub();

        
        void SwitchMode_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_SwitchMode_Request &_request, muas::SwitchMode_Callback _callback, muas::SwitchMode_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Takeoff_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Takeoff_Request &_request, muas::Takeoff_Callback _callback, muas::Takeoff_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Land_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Land_Request &_request, muas::Land_Callback _callback, muas::Land_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void RTL_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_RTL_Request &_request, muas::RTL_Callback _callback, muas::RTL_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Kill_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Kill_Request &_request, muas::Kill_Callback _callback, muas::Kill_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void SetSpeed_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_SetSpeed_Request &_request, muas::SetSpeed_Callback _callback, muas::SetSpeed_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Reposition_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Reposition_Request &_request, muas::Reposition_Callback _callback, muas::Reposition_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,SwitchMode_Callback> SwitchMode_Callbacks;
        std::map<ndn::Name,SwitchMode_Timeout_Callback> SwitchMode_Timeout_Callbacks;
        
        std::map<ndn::Name,Takeoff_Callback> Takeoff_Callbacks;
        std::map<ndn::Name,Takeoff_Timeout_Callback> Takeoff_Timeout_Callbacks;
        
        std::map<ndn::Name,Land_Callback> Land_Callbacks;
        std::map<ndn::Name,Land_Timeout_Callback> Land_Timeout_Callbacks;
        
        std::map<ndn::Name,RTL_Callback> RTL_Callbacks;
        std::map<ndn::Name,RTL_Timeout_Callback> RTL_Timeout_Callbacks;
        
        std::map<ndn::Name,Kill_Callback> Kill_Callbacks;
        std::map<ndn::Name,Kill_Timeout_Callback> Kill_Timeout_Callbacks;
        
        std::map<ndn::Name,SetSpeed_Callback> SetSpeed_Callbacks;
        std::map<ndn::Name,SetSpeed_Timeout_Callback> SetSpeed_Timeout_Callbacks;
        
        std::map<ndn::Name,Reposition_Callback> Reposition_Callbacks;
        std::map<ndn::Name,Reposition_Timeout_Callback> Reposition_Timeout_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif