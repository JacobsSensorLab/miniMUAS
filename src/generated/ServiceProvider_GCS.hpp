#ifndef EXAMPLE_SERVICE_PROVIDER_GCS_HPP
#define EXAMPLE_SERVICE_PROVIDER_GCS_HPP

#include <ndn-service-framework/ServiceProvider.hpp>

#include "./EntityService.hpp"

#include "./AdminService.hpp"



namespace muas
{
    class ServiceProvider_GCS : public ndn_service_framework::ServiceProvider
    {
    public:
        ServiceProvider_GCS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert,ndn::security::Certificate attrAuthorityCertificate, std::string trustSchemaPath);
        virtual ~ServiceProvider_GCS();

    protected:
        virtual void registerServiceInfo() override;

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;


    public:
        
        muas::EntityService m_EntityService;
        
        muas::AdminService m_AdminService;
        
    };
}

#endif