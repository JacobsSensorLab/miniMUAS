#include "./ServiceProvider_GCS.hpp"

namespace muas
{
    NDN_LOG_INIT(muas.ServiceProvider_GCS);
    ServiceProvider_GCS::ServiceProvider_GCS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert, ndn::security::Certificate attrAuthorityCertificate, std::string trustSchemaPath)
        : ndn_service_framework::ServiceProvider(face, group_prefix, identityCert, attrAuthorityCertificate,  trustSchemaPath),
        m_EntityService(*this),m_AdminService(*this)
    {
        
        this->m_serviceNames.push_back("Entity");
        
        this->m_serviceNames.push_back("Admin");
        
        init();
    }

    ServiceProvider_GCS::~ServiceProvider_GCS(){}

    void ServiceProvider_GCS::registerServiceInfo()
    {
        NDN_LOG_INFO("Registering services using NDNSD");
        ndnsd::discovery::Details details;
        
        

        details = {ndn::Name("/Entity/Echo"),
            identity,
            3600,
            time(NULL),
            { {"type", "Entity"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Entity/Echo/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Entity/GetEntityInfo"),
            identity,
            3600,
            time(NULL),
            { {"type", "Entity"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Entity/GetEntityInfo/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Entity/GetPosition"),
            identity,
            3600,
            time(NULL),
            { {"type", "Entity"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Entity/GetPosition/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Entity/GetOrientation"),
            identity,
            3600,
            time(NULL),
            { {"type", "Entity"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Entity/GetOrientation/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Admin/Test"),
            identity,
            3600,
            time(NULL),
            { {"type", "Admin"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Admin/Test/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
    }

    void ServiceProvider_GCS::ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage)
    {
        // log the request
        NDN_LOG_TRACE("Received request from " << RequesterName << " for service " << ServiceName << " function " << FunctionName << " with request id " << RequestID);

        
        if (ServiceName.equals(m_EntityService.serviceName))
        {
            m_EntityService.ConsumeRequest(RequesterName, providerName, ServiceName, FunctionName, RequestID, requestMessage);                                  
        }
        
        if (ServiceName.equals(m_AdminService.serviceName))
        {
            m_AdminService.ConsumeRequest(RequesterName, providerName, ServiceName, FunctionName, RequestID, requestMessage);                                  
        }
        
    }


}