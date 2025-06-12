#ifndef WUASService_HPP
#define WUASService_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>

#include "ndn-service-framework/ServiceProvider.hpp"
#include "ndn-service-framework/ServiceStub.hpp"
#include "ndn-service-framework/Service.hpp"

#include "messages.pb.h"

#include <vector>

namespace muas
{

    
    using QuadRaster_Function = std::function<void(const ndn::Name &, const muas::WUAS_QuadRaster_Request &, muas::WUAS_QuadRaster_Response &)>;
    

    class WUASService : public ndn_service_framework::Service
    {
    public:
        WUASService(ndn_service_framework::ServiceProvider &serviceProvider)
            : ndn_service_framework::Service(serviceProvider),
              serviceName("WUAS")
        {
        }

        virtual ~WUASService();

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;
        
        void QuadRaster(const ndn::Name &requesterIdentity, const muas::WUAS_QuadRaster_Request &_request, muas::WUAS_QuadRaster_Response &_response);
        


    public:
        ndn::Name serviceName;
        
        QuadRaster_Function QuadRaster_Handler;
        
    };
}
#endif