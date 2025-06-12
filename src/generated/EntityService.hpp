#ifndef EntityService_HPP
#define EntityService_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>

#include "ndn-service-framework/ServiceProvider.hpp"
#include "ndn-service-framework/ServiceStub.hpp"
#include "ndn-service-framework/Service.hpp"

#include "messages.pb.h"

#include <vector>

namespace muas
{

    
    using Echo_Function = std::function<void(const ndn::Name &, const muas::Entity_Echo_Request &, muas::Entity_Echo_Response &)>;
    
    using GetEntityInfo_Function = std::function<void(const ndn::Name &, const muas::Entity_GetEntityInfo_Request &, muas::Entity_GetEntityInfo_Response &)>;
    
    using GetPosition_Function = std::function<void(const ndn::Name &, const muas::Entity_GetPosition_Request &, muas::Entity_GetPosition_Response &)>;
    
    using GetOrientation_Function = std::function<void(const ndn::Name &, const muas::Entity_GetOrientation_Request &, muas::Entity_GetOrientation_Response &)>;
    

    class EntityService : public ndn_service_framework::Service
    {
    public:
        EntityService(ndn_service_framework::ServiceProvider &serviceProvider)
            : ndn_service_framework::Service(serviceProvider),
              serviceName("Entity")
        {
        }

        virtual ~EntityService();

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;
        
        void Echo(const ndn::Name &requesterIdentity, const muas::Entity_Echo_Request &_request, muas::Entity_Echo_Response &_response);
        
        void GetEntityInfo(const ndn::Name &requesterIdentity, const muas::Entity_GetEntityInfo_Request &_request, muas::Entity_GetEntityInfo_Response &_response);
        
        void GetPosition(const ndn::Name &requesterIdentity, const muas::Entity_GetPosition_Request &_request, muas::Entity_GetPosition_Response &_response);
        
        void GetOrientation(const ndn::Name &requesterIdentity, const muas::Entity_GetOrientation_Request &_request, muas::Entity_GetOrientation_Response &_response);
        


    public:
        ndn::Name serviceName;
        
        Echo_Function Echo_Handler;
        
        GetEntityInfo_Function GetEntityInfo_Handler;
        
        GetPosition_Function GetPosition_Handler;
        
        GetOrientation_Function GetOrientation_Handler;
        
    };
}
#endif