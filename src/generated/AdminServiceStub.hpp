#ifndef AdminServiceStub_HPP
#define AdminServiceStub_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>


#include "ndn-service-framework/ServiceUser.hpp"
#include "messages.pb.h"

#include <iostream>
#include <string>
#include <regex>

namespace muas
{
    
    using Test_Callback = std::function<void(const muas::Admin_Test_Response &)>;
    using Test_Timeout_Callback = std::function<void(const muas::Admin_Test_Request &)>;
    

    class AdminServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        AdminServiceStub(ndn::Face& face, ndn_service_framework::ServiceUser& user);
        virtual ~AdminServiceStub();

        
        void Test_Async(const std::vector<ndn::Name>& providers, const muas::Admin_Test_Request &_request, muas::Test_Callback _callback, muas::Test_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,Test_Callback> Test_Callbacks;
        std::map<ndn::Name,Test_Timeout_Callback> Test_Timeout_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif