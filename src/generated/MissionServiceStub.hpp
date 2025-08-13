#ifndef MissionServiceStub_HPP
#define MissionServiceStub_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>


#include "ndn-service-framework/ServiceUser.hpp"
#include "messages.pb.h"

#include <iostream>
#include <string>
#include <regex>

namespace muas
{
    
    using GetMissionInfo_Callback = std::function<void(const muas::Mission_GetMissionInfo_Response &)>;
    using GetMissionInfo_Timeout_Callback = std::function<void(const muas::Mission_GetMissionInfo_Request &)>;
    
    using GetItem_Callback = std::function<void(const muas::Mission_GetItem_Response &)>;
    using GetItem_Timeout_Callback = std::function<void(const muas::Mission_GetItem_Request &)>;
    
    using SetItem_Callback = std::function<void(const muas::Mission_SetItem_Response &)>;
    using SetItem_Timeout_Callback = std::function<void(const muas::Mission_SetItem_Request &)>;
    
    using Clear_Callback = std::function<void(const muas::Mission_Clear_Response &)>;
    using Clear_Timeout_Callback = std::function<void(const muas::Mission_Clear_Request &)>;
    
    using Start_Callback = std::function<void(const muas::Mission_Start_Response &)>;
    using Start_Timeout_Callback = std::function<void(const muas::Mission_Start_Request &)>;
    
    using Pause_Callback = std::function<void(const muas::Mission_Pause_Response &)>;
    using Pause_Timeout_Callback = std::function<void(const muas::Mission_Pause_Request &)>;
    
    using Continue_Callback = std::function<void(const muas::Mission_Continue_Response &)>;
    using Continue_Timeout_Callback = std::function<void(const muas::Mission_Continue_Request &)>;
    
    using Terminate_Callback = std::function<void(const muas::Mission_Terminate_Response &)>;
    using Terminate_Timeout_Callback = std::function<void(const muas::Mission_Terminate_Request &)>;
    

    class MissionServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        MissionServiceStub(ndn::Face& face, ndn_service_framework::ServiceUser& user);
        virtual ~MissionServiceStub();

        
        void GetMissionInfo_Async(const std::vector<ndn::Name>& providers, const muas::Mission_GetMissionInfo_Request &_request, muas::GetMissionInfo_Callback _callback, muas::GetMissionInfo_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void GetItem_Async(const std::vector<ndn::Name>& providers, const muas::Mission_GetItem_Request &_request, muas::GetItem_Callback _callback, muas::GetItem_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void SetItem_Async(const std::vector<ndn::Name>& providers, const muas::Mission_SetItem_Request &_request, muas::SetItem_Callback _callback, muas::SetItem_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Clear_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Clear_Request &_request, muas::Clear_Callback _callback, muas::Clear_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Start_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Start_Request &_request, muas::Start_Callback _callback, muas::Start_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Pause_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Pause_Request &_request, muas::Pause_Callback _callback, muas::Pause_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Continue_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Continue_Request &_request, muas::Continue_Callback _callback, muas::Continue_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void Terminate_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Terminate_Request &_request, muas::Terminate_Callback _callback, muas::Terminate_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,GetMissionInfo_Callback> GetMissionInfo_Callbacks;
        std::map<ndn::Name,GetMissionInfo_Timeout_Callback> GetMissionInfo_Timeout_Callbacks;
        
        std::map<ndn::Name,GetItem_Callback> GetItem_Callbacks;
        std::map<ndn::Name,GetItem_Timeout_Callback> GetItem_Timeout_Callbacks;
        
        std::map<ndn::Name,SetItem_Callback> SetItem_Callbacks;
        std::map<ndn::Name,SetItem_Timeout_Callback> SetItem_Timeout_Callbacks;
        
        std::map<ndn::Name,Clear_Callback> Clear_Callbacks;
        std::map<ndn::Name,Clear_Timeout_Callback> Clear_Timeout_Callbacks;
        
        std::map<ndn::Name,Start_Callback> Start_Callbacks;
        std::map<ndn::Name,Start_Timeout_Callback> Start_Timeout_Callbacks;
        
        std::map<ndn::Name,Pause_Callback> Pause_Callbacks;
        std::map<ndn::Name,Pause_Timeout_Callback> Pause_Timeout_Callbacks;
        
        std::map<ndn::Name,Continue_Callback> Continue_Callbacks;
        std::map<ndn::Name,Continue_Timeout_Callback> Continue_Timeout_Callbacks;
        
        std::map<ndn::Name,Terminate_Callback> Terminate_Callbacks;
        std::map<ndn::Name,Terminate_Timeout_Callback> Terminate_Timeout_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif