#include "./ServiceUser_GCS.hpp"

NDN_LOG_INIT(muas.ServiceUser_GCS);

muas::ServiceUser_GCS::ServiceUser_GCS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert,ndn::security::Certificate attrAuthorityCertificate, std::string trustSchemaPath)
    :ndn_service_framework::ServiceUser(face, group_prefix, identityCert, attrAuthorityCertificate, trustSchemaPath),
    
        
    m_EntityServiceStub(*this),
        
    
        
    m_AdminServiceStub(*this),
        
    
        
    m_WUASServiceStub(*this),
        
    
        
    m_IUASServiceStub(*this),
        
    
        
    m_MissionServiceStub(*this),
        
    
        
    m_FlightCtrlServiceStub(*this),
        
    
        
    m_MAVLinkServiceStub(*this),
        
    
        
    m_SensorServiceStub(*this)
        
    
{
    
    this->m_serviceNames.push_back("Entity");
    
    this->m_serviceNames.push_back("Admin");
    
    this->m_serviceNames.push_back("WUAS");
    
    this->m_serviceNames.push_back("IUAS");
    
    this->m_serviceNames.push_back("Mission");
    
    this->m_serviceNames.push_back("FlightCtrl");
    
    this->m_serviceNames.push_back("MAVLink");
    
    this->m_serviceNames.push_back("Sensor");
    
    init();
}

muas::ServiceUser_GCS::~ServiceUser_GCS() {}

void muas::ServiceUser_GCS::OnResponse(const ndn::svs::SVSPubSub::SubscriptionData &subscription)
{
        ndn::Name RequesterName, providerName,ServiceName, FunctionName, RequestId;
        //std::tie(ServiceProviderName, RequesterName, ServiceName, FunctionName, RequestId) =
        auto results=ndn_service_framework::parseResponseName(subscription.name);
        if(!results){
            NDN_LOG_ERROR("parseResponseName failed: " << subscription.name);
            return;
        }
        std::tie(RequesterName, providerName, ServiceName, FunctionName, RequestId) = results.value();
        NDN_LOG_INFO("OnResponse: " << RequesterName << providerName << ServiceName << FunctionName << RequestId);

        // decrypt the request message with nac-abe; if cannot be decrypted
        
        if (ServiceName.equals(m_EntityServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_EntityServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::EntityServiceStub::OnResponseDecryptionSuccessCallback, &m_EntityServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::EntityServiceStub::OnResponseDecryptionSuccessCallback, &m_EntityServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
        if (ServiceName.equals(m_AdminServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_AdminServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::AdminServiceStub::OnResponseDecryptionSuccessCallback, &m_AdminServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::AdminServiceStub::OnResponseDecryptionSuccessCallback, &m_AdminServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
        if (ServiceName.equals(m_WUASServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_WUASServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::WUASServiceStub::OnResponseDecryptionSuccessCallback, &m_WUASServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::WUASServiceStub::OnResponseDecryptionSuccessCallback, &m_WUASServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
        if (ServiceName.equals(m_IUASServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_IUASServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::IUASServiceStub::OnResponseDecryptionSuccessCallback, &m_IUASServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::IUASServiceStub::OnResponseDecryptionSuccessCallback, &m_IUASServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
        if (ServiceName.equals(m_MissionServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_MissionServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::MissionServiceStub::OnResponseDecryptionSuccessCallback, &m_MissionServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::MissionServiceStub::OnResponseDecryptionSuccessCallback, &m_MissionServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
        if (ServiceName.equals(m_FlightCtrlServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_FlightCtrlServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::FlightCtrlServiceStub::OnResponseDecryptionSuccessCallback, &m_FlightCtrlServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::FlightCtrlServiceStub::OnResponseDecryptionSuccessCallback, &m_FlightCtrlServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
        if (ServiceName.equals(m_MAVLinkServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_MAVLinkServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::MAVLinkServiceStub::OnResponseDecryptionSuccessCallback, &m_MAVLinkServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::MAVLinkServiceStub::OnResponseDecryptionSuccessCallback, &m_MAVLinkServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
        if (ServiceName.equals(m_SensorServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_SensorServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::SensorServiceStub::OnResponseDecryptionSuccessCallback, &m_SensorServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::SensorServiceStub::OnResponseDecryptionSuccessCallback, &m_SensorServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_GCS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
}


