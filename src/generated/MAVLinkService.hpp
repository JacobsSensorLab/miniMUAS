#ifndef MAVLinkService_HPP
#define MAVLinkService_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>

#include "ndn-service-framework/ServiceProvider.hpp"
#include "ndn-service-framework/ServiceStub.hpp"
#include "ndn-service-framework/Service.hpp"

#include "messages.pb.h"

#include <vector>

namespace muas
{

    
    using Generic_Function = std::function<void(const ndn::Name &, const muas::MAVLink_Generic_Request &, muas::MAVLink_Generic_Response &)>;
    

    class MAVLinkService : public ndn_service_framework::Service
    {
    public:
        MAVLinkService(ndn_service_framework::ServiceProvider &serviceProvider)
            : ndn_service_framework::Service(serviceProvider),
              serviceName("MAVLink")
        {
        }

        virtual ~MAVLinkService();

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;
        
        void Generic(const ndn::Name &requesterIdentity, const muas::MAVLink_Generic_Request &_request, muas::MAVLink_Generic_Response &_response);
        


    public:
        ndn::Name serviceName;
        
        Generic_Function Generic_Handler;
        
    };
}
#endif