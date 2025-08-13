#ifndef EXAMPLE_SERVICE_USER_WUAS_HPP
#define EXAMPLE_SERVICE_USER_WUAS_HPP

#include <ndn-service-framework/ServiceUser.hpp>
#include <ndn-service-framework/NDNSFMessages.hpp>

#include "./EntityServiceStub.hpp"

#include "./AdminServiceStub.hpp"

#include "./IUASServiceStub.hpp"

#include "./FlightCtrlServiceStub.hpp"

#include "./MAVLinkServiceStub.hpp"

#include "./SensorServiceStub.hpp"



namespace muas
{
    class ServiceUser_WUAS : public ndn_service_framework::ServiceUser
    {
    public:
        ServiceUser_WUAS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert, ndn::security::Certificate attrAuthorityCertificate,std::string trustSchemaPath);
        virtual ~ServiceUser_WUAS();

        
        void Echo_Async(const std::vector<ndn::Name>& providers, const muas::Entity_Echo_Request &_request, muas::Echo_Callback _callback,  muas::Echo_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_EntityServiceStub.Echo_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void GetEntityInfo_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetEntityInfo_Request &_request, muas::GetEntityInfo_Callback _callback,  muas::GetEntityInfo_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_EntityServiceStub.GetEntityInfo_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void GetPosition_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetPosition_Request &_request, muas::GetPosition_Callback _callback,  muas::GetPosition_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_EntityServiceStub.GetPosition_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void GetOrientation_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetOrientation_Request &_request, muas::GetOrientation_Callback _callback,  muas::GetOrientation_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_EntityServiceStub.GetOrientation_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void Test_Async(const std::vector<ndn::Name>& providers, const muas::Admin_Test_Request &_request, muas::Test_Callback _callback,  muas::Test_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_AdminServiceStub.Test_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void PointOrbit_Async(const std::vector<ndn::Name>& providers, const muas::IUAS_PointOrbit_Request &_request, muas::PointOrbit_Callback _callback,  muas::PointOrbit_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_IUASServiceStub.PointOrbit_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void PointHover_Async(const std::vector<ndn::Name>& providers, const muas::IUAS_PointHover_Request &_request, muas::PointHover_Callback _callback,  muas::PointHover_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_IUASServiceStub.PointHover_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void SwitchMode_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_SwitchMode_Request &_request, muas::SwitchMode_Callback _callback,  muas::SwitchMode_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_FlightCtrlServiceStub.SwitchMode_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void Takeoff_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Takeoff_Request &_request, muas::Takeoff_Callback _callback,  muas::Takeoff_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_FlightCtrlServiceStub.Takeoff_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void Land_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Land_Request &_request, muas::Land_Callback _callback,  muas::Land_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_FlightCtrlServiceStub.Land_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void RTL_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_RTL_Request &_request, muas::RTL_Callback _callback,  muas::RTL_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_FlightCtrlServiceStub.RTL_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void Kill_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Kill_Request &_request, muas::Kill_Callback _callback,  muas::Kill_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_FlightCtrlServiceStub.Kill_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void SetSpeed_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_SetSpeed_Request &_request, muas::SetSpeed_Callback _callback,  muas::SetSpeed_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_FlightCtrlServiceStub.SetSpeed_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void Reposition_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Reposition_Request &_request, muas::Reposition_Callback _callback,  muas::Reposition_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_FlightCtrlServiceStub.Reposition_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void Generic_Async(const std::vector<ndn::Name>& providers, const muas::MAVLink_Generic_Request &_request, muas::Generic_Callback _callback,  muas::Generic_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_MAVLinkServiceStub.Generic_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void GetSensorInfo_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_GetSensorInfo_Request &_request, muas::GetSensorInfo_Callback _callback,  muas::GetSensorInfo_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_SensorServiceStub.GetSensorInfo_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void CaptureSingle_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_CaptureSingle_Request &_request, muas::CaptureSingle_Callback _callback,  muas::CaptureSingle_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_SensorServiceStub.CaptureSingle_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        
        void CapturePeriodic_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_CapturePeriodic_Request &_request, muas::CapturePeriodic_Callback _callback,  muas::CapturePeriodic_Timeout_Callback _timeout_callback, int timeout_ms,  const size_t strategy = ndn_service_framework::tlv::FirstResponding)
        {
            m_SensorServiceStub.CapturePeriodic_Async(providers, _request, _callback, _timeout_callback, timeout_ms, strategy);
        }
        

      
    protected:
        
        void OnResponse(const ndn::svs::SVSPubSub::SubscriptionData &subscription) override;
        
    private:
        
        muas::EntityServiceStub m_EntityServiceStub;
        
        muas::AdminServiceStub m_AdminServiceStub;
        
        muas::IUASServiceStub m_IUASServiceStub;
        
        muas::FlightCtrlServiceStub m_FlightCtrlServiceStub;
        
        muas::MAVLinkServiceStub m_MAVLinkServiceStub;
        
        muas::SensorServiceStub m_SensorServiceStub;
        
    };
}

#endif