#ifndef EntityServiceStub_HPP
#define EntityServiceStub_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>


#include "ndn-service-framework/ServiceUser.hpp"
#include "messages.pb.h"

#include <iostream>
#include <string>
#include <regex>

namespace muas
{
    
    using Echo_Callback = std::function<void(const muas::Entity_Echo_Response &)>;
    using Echo_Timeout_Callback = std::function<void(const muas::Entity_Echo_Request &)>;
    
    using GetEntityInfo_Callback = std::function<void(const muas::Entity_GetEntityInfo_Response &)>;
    using GetEntityInfo_Timeout_Callback = std::function<void(const muas::Entity_GetEntityInfo_Request &)>;
    
    using GetPosition_Callback = std::function<void(const muas::Entity_GetPosition_Response &)>;
    using GetPosition_Timeout_Callback = std::function<void(const muas::Entity_GetPosition_Request &)>;
    
    using GetOrientation_Callback = std::function<void(const muas::Entity_GetOrientation_Response &)>;
    using GetOrientation_Timeout_Callback = std::function<void(const muas::Entity_GetOrientation_Request &)>;
    

    class EntityServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        EntityServiceStub(ndn::Face& face, ndn_service_framework::ServiceUser& user);
        virtual ~EntityServiceStub();

        
        void Echo_Async(const std::vector<ndn::Name>& providers, const muas::Entity_Echo_Request &_request, muas::Echo_Callback _callback, muas::Echo_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void GetEntityInfo_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetEntityInfo_Request &_request, muas::GetEntityInfo_Callback _callback, muas::GetEntityInfo_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void GetPosition_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetPosition_Request &_request, muas::GetPosition_Callback _callback, muas::GetPosition_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
        
        void GetOrientation_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetOrientation_Request &_request, muas::GetOrientation_Callback _callback, muas::GetOrientation_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,Echo_Callback> Echo_Callbacks;
        std::map<ndn::Name,Echo_Timeout_Callback> Echo_Timeout_Callbacks;
        
        std::map<ndn::Name,GetEntityInfo_Callback> GetEntityInfo_Callbacks;
        std::map<ndn::Name,GetEntityInfo_Timeout_Callback> GetEntityInfo_Timeout_Callbacks;
        
        std::map<ndn::Name,GetPosition_Callback> GetPosition_Callbacks;
        std::map<ndn::Name,GetPosition_Timeout_Callback> GetPosition_Timeout_Callbacks;
        
        std::map<ndn::Name,GetOrientation_Callback> GetOrientation_Callbacks;
        std::map<ndn::Name,GetOrientation_Timeout_Callback> GetOrientation_Timeout_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif