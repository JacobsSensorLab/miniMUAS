#ifndef EXAMPLE_SERVICE_PROVIDER_IUAS_HPP
#define EXAMPLE_SERVICE_PROVIDER_IUAS_HPP

#include <ndn-service-framework/ServiceProvider.hpp>

#include "./EntityService.hpp"

#include "./IUASService.hpp"

#include "./MissionService.hpp"

#include "./FlightCtrlService.hpp"

#include "./MAVLinkService.hpp"

#include "./SensorService.hpp"



namespace muas
{
    class ServiceProvider_IUAS : public ndn_service_framework::ServiceProvider
    {
    public:
        ServiceProvider_IUAS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert,ndn::security::Certificate attrAuthorityCertificate, std::string trustSchemaPath);
        virtual ~ServiceProvider_IUAS();

    protected:
        virtual void registerServiceInfo() override;

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;


    public:
        
        muas::EntityService m_EntityService;
        
        muas::IUASService m_IUASService;
        
        muas::MissionService m_MissionService;
        
        muas::FlightCtrlService m_FlightCtrlService;
        
        muas::MAVLinkService m_MAVLinkService;
        
        muas::SensorService m_SensorService;
        
    };
}

#endif