#ifndef EXAMPLE_SERVICE_USER_IUAS_HPP
#define EXAMPLE_SERVICE_USER_IUAS_HPP

#include <ndn-service-framework/ServiceUser.hpp>
#include <ndn-service-framework/NDNSFMessages.hpp>

#include "./EntityServiceStub.hpp"

#include "./AdminServiceStub.hpp"



namespace muas
{
    class ServiceUser_IUAS : public ndn_service_framework::ServiceUser
    {
    public:
        ServiceUser_IUAS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert, ndn::security::Certificate attrAuthorityCertificate,std::string trustSchemaPath);
        virtual ~ServiceUser_IUAS();

        
        void Echo_Async(const std::vector<ndn::Name>& providers, const muas::Entity_Echo_Request &_request, muas::Echo_Callback _callback,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_EntityServiceStub.Echo_Async(providers, _request, _callback, strategy);
        }
        
        void GetEntityInfo_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetEntityInfo_Request &_request, muas::GetEntityInfo_Callback _callback,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_EntityServiceStub.GetEntityInfo_Async(providers, _request, _callback, strategy);
        }
        
        void GetPosition_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetPosition_Request &_request, muas::GetPosition_Callback _callback,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_EntityServiceStub.GetPosition_Async(providers, _request, _callback, strategy);
        }
        
        void GetOrientation_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetOrientation_Request &_request, muas::GetOrientation_Callback _callback,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_EntityServiceStub.GetOrientation_Async(providers, _request, _callback, strategy);
        }
        
        void Test_Async(const std::vector<ndn::Name>& providers, const muas::Admin_Test_Request &_request, muas::Test_Callback _callback,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_AdminServiceStub.Test_Async(providers, _request, _callback, strategy);
        }
        

      
    protected:
        
        void OnResponse(const ndn::svs::SVSPubSub::SubscriptionData &subscription) override;
        
    private:
        
        muas::EntityServiceStub m_EntityServiceStub;
        
        muas::AdminServiceStub m_AdminServiceStub;
        
    };
}

#endif