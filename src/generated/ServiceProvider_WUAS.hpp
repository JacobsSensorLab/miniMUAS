#ifndef EXAMPLE_SERVICE_PROVIDER_WUAS_HPP
#define EXAMPLE_SERVICE_PROVIDER_WUAS_HPP

#include <ndn-service-framework/ServiceProvider.hpp>

#include "./EntityService.hpp"

#include "./AdminService.hpp"

#include "./MissionService.hpp"

#include "./WUASService.hpp"

#include "./FlightCtrlService.hpp"

#include "./MAVLinkService.hpp"

#include "./SensorService.hpp"



namespace muas
{
    class ServiceProvider_WUAS : public ndn_service_framework::ServiceProvider
    {
    public:
        ServiceProvider_WUAS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert,ndn::security::Certificate attrAuthorityCertificate, std::string trustSchemaPath);
        virtual ~ServiceProvider_WUAS();

    protected:
        virtual void registerServiceInfo() override;

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;


    public:
        
        muas::EntityService m_EntityService;
        
        muas::AdminService m_AdminService;
        
        muas::MissionService m_MissionService;
        
        muas::WUASService m_WUASService;
        
        muas::FlightCtrlService m_FlightCtrlService;
        
        muas::MAVLinkService m_MAVLinkService;
        
        muas::SensorService m_SensorService;
        
    };
}

#endif