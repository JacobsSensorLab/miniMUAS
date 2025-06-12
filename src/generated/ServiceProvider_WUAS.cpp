#include "./ServiceProvider_WUAS.hpp"

namespace muas
{
    NDN_LOG_INIT(muas.ServiceProvider_WUAS);
    ServiceProvider_WUAS::ServiceProvider_WUAS(ndn::Face& face, ndn::Name group_prefix, ndn::security::Certificate identityCert, ndn::security::Certificate attrAuthorityCertificate, std::string trustSchemaPath)
        : ndn_service_framework::ServiceProvider(face, group_prefix, identityCert, attrAuthorityCertificate,  trustSchemaPath),
        m_EntityService(*this),m_AdminService(*this),m_MissionService(*this),m_WUASService(*this),m_FlightCtrlService(*this),m_MAVLinkService(*this),m_SensorService(*this)
    {
        
        this->m_serviceNames.push_back("Entity");
        
        this->m_serviceNames.push_back("Admin");
        
        this->m_serviceNames.push_back("Mission");
        
        this->m_serviceNames.push_back("WUAS");
        
        this->m_serviceNames.push_back("FlightCtrl");
        
        this->m_serviceNames.push_back("MAVLink");
        
        this->m_serviceNames.push_back("Sensor");
        
        init();
    }

    ServiceProvider_WUAS::~ServiceProvider_WUAS(){}

    void ServiceProvider_WUAS::registerServiceInfo()
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
        
        

        details = {ndn::Name("/Mission/GetMissionInfo"),
            identity,
            3600,
            time(NULL),
            { {"type", "Mission"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Mission/GetMissionInfo/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Mission/GetItem"),
            identity,
            3600,
            time(NULL),
            { {"type", "Mission"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Mission/GetItem/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Mission/SetItem"),
            identity,
            3600,
            time(NULL),
            { {"type", "Mission"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Mission/SetItem/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Mission/Clear"),
            identity,
            3600,
            time(NULL),
            { {"type", "Mission"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Mission/Clear/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Mission/Start"),
            identity,
            3600,
            time(NULL),
            { {"type", "Mission"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Mission/Start/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Mission/Pause"),
            identity,
            3600,
            time(NULL),
            { {"type", "Mission"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Mission/Pause/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Mission/Continue"),
            identity,
            3600,
            time(NULL),
            { {"type", "Mission"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Mission/Continue/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Mission/Terminate"),
            identity,
            3600,
            time(NULL),
            { {"type", "Mission"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Mission/Terminate/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/WUAS/QuadRaster"),
            identity,
            3600,
            time(NULL),
            { {"type", "WUAS"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/WUAS/QuadRaster/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/FlightCtrl/SwitchMode"),
            identity,
            3600,
            time(NULL),
            { {"type", "FlightCtrl"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/FlightCtrl/SwitchMode/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/FlightCtrl/Takeoff"),
            identity,
            3600,
            time(NULL),
            { {"type", "FlightCtrl"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/FlightCtrl/Takeoff/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/FlightCtrl/Land"),
            identity,
            3600,
            time(NULL),
            { {"type", "FlightCtrl"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/FlightCtrl/Land/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/FlightCtrl/RTL"),
            identity,
            3600,
            time(NULL),
            { {"type", "FlightCtrl"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/FlightCtrl/RTL/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/FlightCtrl/Kill"),
            identity,
            3600,
            time(NULL),
            { {"type", "FlightCtrl"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/FlightCtrl/Kill/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/FlightCtrl/SetSpeed"),
            identity,
            3600,
            time(NULL),
            { {"type", "FlightCtrl"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/FlightCtrl/SetSpeed/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/FlightCtrl/Reposition"),
            identity,
            3600,
            time(NULL),
            { {"type", "FlightCtrl"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/FlightCtrl/Reposition/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/MAVLink/Generic"),
            identity,
            3600,
            time(NULL),
            { {"type", "MAVLink"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/MAVLink/Generic/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Sensor/GetSensorInfo"),
            identity,
            3600,
            time(NULL),
            { {"type", "Sensor"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Sensor/GetSensorInfo/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Sensor/CaptureSingle"),
            identity,
            3600,
            time(NULL),
            { {"type", "Sensor"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Sensor/CaptureSingle/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
        

        details = {ndn::Name("/Sensor/CapturePeriodic"),
            identity,
            3600,
            time(NULL),
            { {"type", "Sensor"}, {"version", "1.0.0"}, {"tokenName", identity.toUri()+"/NDNSF/TOKEN/Sensor/CapturePeriodic/0"} }};
        m_ServiceDiscovery.publishServiceDetail(details);
        UpdateUPTWithServiceMetaInfo(details);
        
    }

    void ServiceProvider_WUAS::ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage)
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
        
        if (ServiceName.equals(m_MissionService.serviceName))
        {
            m_MissionService.ConsumeRequest(RequesterName, providerName, ServiceName, FunctionName, RequestID, requestMessage);                                  
        }
        
        if (ServiceName.equals(m_WUASService.serviceName))
        {
            m_WUASService.ConsumeRequest(RequesterName, providerName, ServiceName, FunctionName, RequestID, requestMessage);                                  
        }
        
        if (ServiceName.equals(m_FlightCtrlService.serviceName))
        {
            m_FlightCtrlService.ConsumeRequest(RequesterName, providerName, ServiceName, FunctionName, RequestID, requestMessage);                                  
        }
        
        if (ServiceName.equals(m_MAVLinkService.serviceName))
        {
            m_MAVLinkService.ConsumeRequest(RequesterName, providerName, ServiceName, FunctionName, RequestID, requestMessage);                                  
        }
        
        if (ServiceName.equals(m_SensorService.serviceName))
        {
            m_SensorService.ConsumeRequest(RequesterName, providerName, ServiceName, FunctionName, RequestID, requestMessage);                                  
        }
        
    }


}