#ifndef AdminService_HPP
#define AdminService_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>

#include "ndn-service-framework/ServiceProvider.hpp"
#include "ndn-service-framework/ServiceStub.hpp"
#include "ndn-service-framework/Service.hpp"

#include "messages.pb.h"

#include <vector>

namespace muas
{

    
    using Test_Function = std::function<void(const ndn::Name &, const muas::Admin_Test_Request &, muas::Admin_Test_Response &)>;
    

    class AdminService : public ndn_service_framework::Service
    {
    public:
        AdminService(ndn_service_framework::ServiceProvider &serviceProvider)
            : ndn_service_framework::Service(serviceProvider),
              serviceName("Admin")
        {
        }

        virtual ~AdminService();

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;
        
        void Test(const ndn::Name &requesterIdentity, const muas::Admin_Test_Request &_request, muas::Admin_Test_Response &_response);
        


    public:
        ndn::Name serviceName;
        
        Test_Function Test_Handler;
        
    };
}
#endif