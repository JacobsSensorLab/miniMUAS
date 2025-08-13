#ifndef WUASServiceStub_HPP
#define WUASServiceStub_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>


#include "ndn-service-framework/ServiceUser.hpp"
#include "messages.pb.h"

#include <iostream>
#include <string>
#include <regex>

namespace muas
{
    
    using QuadRaster_Callback = std::function<void(const muas::WUAS_QuadRaster_Response &)>;
    using QuadRaster_Timeout_Callback = std::function<void(const muas::WUAS_QuadRaster_Request &)>;
    

    class WUASServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        WUASServiceStub(ndn::Face& face, ndn_service_framework::ServiceUser& user);
        virtual ~WUASServiceStub();

        
        void QuadRaster_Async(const std::vector<ndn::Name>& providers, const muas::WUAS_QuadRaster_Request &_request, muas::QuadRaster_Callback _callback, muas::QuadRaster_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,QuadRaster_Callback> QuadRaster_Callbacks;
        std::map<ndn::Name,QuadRaster_Timeout_Callback> QuadRaster_Timeout_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif