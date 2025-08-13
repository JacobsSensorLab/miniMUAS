#include "./SensorServiceStub.hpp"

NDN_LOG_INIT(muas.SensorServiceStub);

muas::SensorServiceStub::SensorServiceStub(ndn_service_framework::ServiceUser &user)
    : ndn_service_framework::ServiceStub(user),
      serviceName("Sensor")
{
}

muas::SensorServiceStub::~SensorServiceStub(){}


void muas::SensorServiceStub::GetSensorInfo_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_GetSensorInfo_Request &_request, muas::GetSensorInfo_Callback _callback, muas::GetSensorInfo_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("GetSensorInfo_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::SensorCtrl_GetSensorInfo_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Sensor"), ndn::Name("GetSensorInfo"), requestId, payload, strategy);
    GetSensorInfo_Callbacks.emplace(requestId, _callback);
    GetSensorInfo_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->GetSensorInfo_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}

void muas::SensorServiceStub::CaptureSingle_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_CaptureSingle_Request &_request, muas::CaptureSingle_Callback _callback, muas::CaptureSingle_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("CaptureSingle_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::SensorCtrl_CaptureSingle_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Sensor"), ndn::Name("CaptureSingle"), requestId, payload, strategy);
    CaptureSingle_Callbacks.emplace(requestId, _callback);
    CaptureSingle_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->CaptureSingle_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}

void muas::SensorServiceStub::CapturePeriodic_Async(const std::vector<ndn::Name>& providers, const muas::SensorCtrl_CapturePeriodic_Request &_request, muas::CapturePeriodic_Callback _callback, muas::CapturePeriodic_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("CapturePeriodic_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::SensorCtrl_CapturePeriodic_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Sensor"), ndn::Name("CapturePeriodic"), requestId, payload, strategy);
    CapturePeriodic_Callbacks.emplace(requestId, _callback);
    CapturePeriodic_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->CapturePeriodic_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}


void muas::SensorServiceStub::OnResponseDecryptionSuccessCallback(const ndn::Name &serviceProviderName, const ndn::Name &ServiceName, const ndn::Name &FunctionName, const ndn::Name &RequestID, const ndn::Buffer &buffer)
{
    NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: " << serviceProviderName << ServiceName << FunctionName << RequestID);

    // parse Response Message from buffer
    ndn_service_framework::ResponseMessage responseMessage;
    responseMessage.WireDecode(ndn::Block(buffer));
    responseMessage.getErrorInfo();

    ndn::Buffer payload = responseMessage.getPayload();

    
    if (ServiceName.equals(ndn::Name("Sensor")) & FunctionName.equals(ndn::Name("GetSensorInfo")))
    {
        
        // SensorService.GetSensorInfo()
        muas::SensorCtrl_GetSensorInfo_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::SensorCtrl_GetSensorInfo_Response parse success");
            auto it = GetSensorInfo_Callbacks.find(RequestID);
            if (it != GetSensorInfo_Callbacks.end())
            {
                it->second(_response);
                // find strategy in the strategyMap using RequestID, and check whether it's ndn_service_framework::tlv:NoCoordination
                // if yes, then remove the callback from the map, otherwise, do nothing.
                auto strategyIt = strategyMap.find(RequestID);
                if (strategyIt != strategyMap.end())
                {
                    if (strategyIt->second != ndn_service_framework::tlv::NoCoordination)
                    {
                        strategyMap.erase(strategyIt);
                        GetSensorInfo_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    GetSensorInfo_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::SensorCtrl_GetSensorInfo_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Sensor")) & FunctionName.equals(ndn::Name("CaptureSingle")))
    {
        
        // SensorService.CaptureSingle()
        muas::SensorCtrl_CaptureSingle_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::SensorCtrl_CaptureSingle_Response parse success");
            auto it = CaptureSingle_Callbacks.find(RequestID);
            if (it != CaptureSingle_Callbacks.end())
            {
                it->second(_response);
                // find strategy in the strategyMap using RequestID, and check whether it's ndn_service_framework::tlv:NoCoordination
                // if yes, then remove the callback from the map, otherwise, do nothing.
                auto strategyIt = strategyMap.find(RequestID);
                if (strategyIt != strategyMap.end())
                {
                    if (strategyIt->second != ndn_service_framework::tlv::NoCoordination)
                    {
                        strategyMap.erase(strategyIt);
                        CaptureSingle_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    CaptureSingle_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::SensorCtrl_CaptureSingle_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Sensor")) & FunctionName.equals(ndn::Name("CapturePeriodic")))
    {
        
        // SensorService.CapturePeriodic()
        muas::SensorCtrl_CapturePeriodic_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::SensorCtrl_CapturePeriodic_Response parse success");
            auto it = CapturePeriodic_Callbacks.find(RequestID);
            if (it != CapturePeriodic_Callbacks.end())
            {
                it->second(_response);
                // find strategy in the strategyMap using RequestID, and check whether it's ndn_service_framework::tlv:NoCoordination
                // if yes, then remove the callback from the map, otherwise, do nothing.
                auto strategyIt = strategyMap.find(RequestID);
                if (strategyIt != strategyMap.end())
                {
                    if (strategyIt->second != ndn_service_framework::tlv::NoCoordination)
                    {
                        strategyMap.erase(strategyIt);
                        CapturePeriodic_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    CapturePeriodic_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::SensorCtrl_CapturePeriodic_Response parse failed");
        }
    }
    
}