#ifndef IUASService_HPP
#define IUASService_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>

#include "ndn-service-framework/ServiceProvider.hpp"
#include "ndn-service-framework/ServiceStub.hpp"
#include "ndn-service-framework/Service.hpp"

#include "messages.pb.h"

#include <vector>

namespace muas
{

    
    using PointOrbit_Function = std::function<void(const ndn::Name &, const muas::IUAS_PointOrbit_Request &, muas::IUAS_PointOrbit_Response &)>;
    
    using PointHover_Function = std::function<void(const ndn::Name &, const muas::IUAS_PointHover_Request &, muas::IUAS_PointHover_Response &)>;
    

    class IUASService : public ndn_service_framework::Service
    {
    public:
        IUASService(ndn_service_framework::ServiceProvider &serviceProvider)
            : ndn_service_framework::Service(serviceProvider),
              serviceName("IUAS")
        {
        }

        virtual ~IUASService();

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;
        
        void PointOrbit(const ndn::Name &requesterIdentity, const muas::IUAS_PointOrbit_Request &_request, muas::IUAS_PointOrbit_Response &_response);
        
        void PointHover(const ndn::Name &requesterIdentity, const muas::IUAS_PointHover_Request &_request, muas::IUAS_PointHover_Response &_response);
        


    public:
        ndn::Name serviceName;
        
        PointOrbit_Function PointOrbit_Handler;
        
        PointHover_Function PointHover_Handler;
        
    };
}
#endif