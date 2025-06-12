#ifndef MAVLinkServiceStub_HPP
#define MAVLinkServiceStub_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>


#include "ndn-service-framework/ServiceUser.hpp"
#include "messages.pb.h"

#include <iostream>
#include <string>
#include <regex>

namespace muas
{
    
    using Generic_Callback = std::function<void(const muas::MAVLink_Generic_Response &)>;
    

    class MAVLinkServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        MAVLinkServiceStub(ndn_service_framework::ServiceUser& user);
        virtual ~MAVLinkServiceStub();

        
        void Generic_Async(const std::vector<ndn::Name>& providers, const muas::MAVLink_Generic_Request &_request, muas::Generic_Callback _callback,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,Generic_Callback> Generic_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif