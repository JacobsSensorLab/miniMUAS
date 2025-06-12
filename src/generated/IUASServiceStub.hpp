#ifndef IUASServiceStub_HPP
#define IUASServiceStub_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>


#include "ndn-service-framework/ServiceUser.hpp"
#include "messages.pb.h"

#include <iostream>
#include <string>
#include <regex>

namespace muas
{
    
    using PointOrbit_Callback = std::function<void(const muas::IUAS_PointOrbit_Response &)>;
    
    using PointHover_Callback = std::function<void(const muas::IUAS_PointHover_Response &)>;
    

    class IUASServiceStub : public ndn_service_framework::ServiceStub
    {
    public:
        IUASServiceStub(ndn_service_framework::ServiceUser& user);
        virtual ~IUASServiceStub();

        
        void PointOrbit_Async(const std::vector<ndn::Name>& providers, const muas::IUAS_PointOrbit_Request &_request, muas::PointOrbit_Callback _callback,  const size_t strategy);
        
        void PointHover_Async(const std::vector<ndn::Name>& providers, const muas::IUAS_PointHover_Request &_request, muas::PointHover_Callback _callback,  const size_t strategy);
              

        void OnResponseDecryptionSuccessCallback(const ndn::Name& serviceProviderName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID,const ndn::Buffer &buffer) override;


    public:
        std::map<ndn::Name,size_t> strategyMap;
        
        std::map<ndn::Name,PointOrbit_Callback> PointOrbit_Callbacks;
        
        std::map<ndn::Name,PointHover_Callback> PointHover_Callbacks;
        
        ndn::Name serviceName;
    };
}

#endif