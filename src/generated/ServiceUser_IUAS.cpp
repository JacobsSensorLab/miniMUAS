#include "./ServiceUser_IUAS.hpp"

NDN_LOG_INIT(muas.ServiceUser_IUAS);

muas::ServiceUser_IUAS::ServiceUser_IUAS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert,ndn::security::Certificate attrAuthorityCertificate, std::string trustSchemaPath)
    :ndn_service_framework::ServiceUser(face, group_prefix, identityCert, attrAuthorityCertificate, trustSchemaPath),
    
        
    m_EntityServiceStub(*this),
        
    
        
    m_AdminServiceStub(*this)
        
    
{
    
    this->m_serviceNames.push_back("Entity");
    
    this->m_serviceNames.push_back("Admin");
    
    init();
}

muas::ServiceUser_IUAS::~ServiceUser_IUAS() {}

void muas::ServiceUser_IUAS::OnResponse(const ndn::svs::SVSPubSub::SubscriptionData &subscription)
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
                                    std::bind(&ServiceUser_IUAS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::EntityServiceStub::OnResponseDecryptionSuccessCallback, &m_EntityServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_IUAS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
        if (ServiceName.equals(m_AdminServiceStub.serviceName))
        {
            NDN_LOG_INFO("Response for : "  << m_AdminServiceStub.serviceName);
            if(subscription.data.size() > 0){
                nacConsumer.consume(subscription.name,
                                    ndn::Block(subscription.data),
                                    std::bind(&muas::AdminServiceStub::OnResponseDecryptionSuccessCallback, &m_AdminServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_IUAS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }else{
                nacConsumer.consume(subscription.name,
                                    std::bind(&muas::AdminServiceStub::OnResponseDecryptionSuccessCallback, &m_AdminServiceStub, providerName, ServiceName, FunctionName, RequestId, _1),
                                    std::bind(&ServiceUser_IUAS::OnResponseDecryptionErrorCallback, this, providerName, ServiceName, FunctionName, RequestId, _1));
            }
        }
        
}


