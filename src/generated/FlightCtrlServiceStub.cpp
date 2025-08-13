#include "./FlightCtrlServiceStub.hpp"

NDN_LOG_INIT(muas.FlightCtrlServiceStub);

muas::FlightCtrlServiceStub::FlightCtrlServiceStub(ndn::Face& face, ndn_service_framework::ServiceUser &user)
    : ndn_service_framework::ServiceStub(face, user),
      serviceName("FlightCtrl")
{
}

muas::FlightCtrlServiceStub::~FlightCtrlServiceStub(){}


void muas::FlightCtrlServiceStub::SwitchMode_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_SwitchMode_Request &_request, muas::SwitchMode_Callback _callback, muas::SwitchMode_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("SwitchMode_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::FlightCtrl_SwitchMode_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("FlightCtrl"), ndn::Name("SwitchMode"), requestId, payload, strategy);
    SwitchMode_Callbacks.emplace(requestId, _callback);
    SwitchMode_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request] { 
        // time out
        this->SwitchMode_Callbacks.erase(requestId);
        // check if timeout_callback is still valid
        auto it = SwitchMode_Timeout_Callbacks.find(requestId);
        if (it != SwitchMode_Timeout_Callbacks.end()) {
            it->second(_request);
        }
    });
}

void muas::FlightCtrlServiceStub::Takeoff_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Takeoff_Request &_request, muas::Takeoff_Callback _callback, muas::Takeoff_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("Takeoff_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::FlightCtrl_Takeoff_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("FlightCtrl"), ndn::Name("Takeoff"), requestId, payload, strategy);
    Takeoff_Callbacks.emplace(requestId, _callback);
    Takeoff_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request] { 
        // time out
        this->Takeoff_Callbacks.erase(requestId);
        // check if timeout_callback is still valid
        auto it = Takeoff_Timeout_Callbacks.find(requestId);
        if (it != Takeoff_Timeout_Callbacks.end()) {
            it->second(_request);
        }
    });
}

void muas::FlightCtrlServiceStub::Land_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Land_Request &_request, muas::Land_Callback _callback, muas::Land_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("Land_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::FlightCtrl_Land_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("FlightCtrl"), ndn::Name("Land"), requestId, payload, strategy);
    Land_Callbacks.emplace(requestId, _callback);
    Land_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request] { 
        // time out
        this->Land_Callbacks.erase(requestId);
        // check if timeout_callback is still valid
        auto it = Land_Timeout_Callbacks.find(requestId);
        if (it != Land_Timeout_Callbacks.end()) {
            it->second(_request);
        }
    });
}

void muas::FlightCtrlServiceStub::RTL_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_RTL_Request &_request, muas::RTL_Callback _callback, muas::RTL_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("RTL_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::FlightCtrl_RTL_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("FlightCtrl"), ndn::Name("RTL"), requestId, payload, strategy);
    RTL_Callbacks.emplace(requestId, _callback);
    RTL_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request] { 
        // time out
        this->RTL_Callbacks.erase(requestId);
        // check if timeout_callback is still valid
        auto it = RTL_Timeout_Callbacks.find(requestId);
        if (it != RTL_Timeout_Callbacks.end()) {
            it->second(_request);
        }
    });
}

void muas::FlightCtrlServiceStub::Kill_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Kill_Request &_request, muas::Kill_Callback _callback, muas::Kill_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("Kill_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::FlightCtrl_Kill_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("FlightCtrl"), ndn::Name("Kill"), requestId, payload, strategy);
    Kill_Callbacks.emplace(requestId, _callback);
    Kill_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request] { 
        // time out
        this->Kill_Callbacks.erase(requestId);
        // check if timeout_callback is still valid
        auto it = Kill_Timeout_Callbacks.find(requestId);
        if (it != Kill_Timeout_Callbacks.end()) {
            it->second(_request);
        }
    });
}

void muas::FlightCtrlServiceStub::SetSpeed_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_SetSpeed_Request &_request, muas::SetSpeed_Callback _callback, muas::SetSpeed_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("SetSpeed_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::FlightCtrl_SetSpeed_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("FlightCtrl"), ndn::Name("SetSpeed"), requestId, payload, strategy);
    SetSpeed_Callbacks.emplace(requestId, _callback);
    SetSpeed_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request] { 
        // time out
        this->SetSpeed_Callbacks.erase(requestId);
        // check if timeout_callback is still valid
        auto it = SetSpeed_Timeout_Callbacks.find(requestId);
        if (it != SetSpeed_Timeout_Callbacks.end()) {
            it->second(_request);
        }
    });
}

void muas::FlightCtrlServiceStub::Reposition_Async(const std::vector<ndn::Name>& providers, const muas::FlightCtrl_Reposition_Request &_request, muas::Reposition_Callback _callback, muas::Reposition_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("Reposition_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::FlightCtrl_Reposition_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("FlightCtrl"), ndn::Name("Reposition"), requestId, payload, strategy);
    Reposition_Callbacks.emplace(requestId, _callback);
    Reposition_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request] { 
        // time out
        this->Reposition_Callbacks.erase(requestId);
        // check if timeout_callback is still valid
        auto it = Reposition_Timeout_Callbacks.find(requestId);
        if (it != Reposition_Timeout_Callbacks.end()) {
            it->second(_request);
        }
    });
}


void muas::FlightCtrlServiceStub::OnResponseDecryptionSuccessCallback(const ndn::Name &serviceProviderName, const ndn::Name &ServiceName, const ndn::Name &FunctionName, const ndn::Name &RequestID, const ndn::Buffer &buffer)
{
    NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: " << serviceProviderName << ServiceName << FunctionName << RequestID);

    // parse Response Message from buffer
    ndn_service_framework::ResponseMessage responseMessage;
    responseMessage.WireDecode(ndn::Block(buffer));
    responseMessage.getErrorInfo();

    ndn::Buffer payload = responseMessage.getPayload();

    
    if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("SwitchMode")))
    {
        
        // FlightCtrlService.SwitchMode()
        muas::FlightCtrl_SwitchMode_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::FlightCtrl_SwitchMode_Response parse success");
            auto it = SwitchMode_Callbacks.find(RequestID);
            if (it != SwitchMode_Callbacks.end())
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
                        SwitchMode_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    SwitchMode_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::FlightCtrl_SwitchMode_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("Takeoff")))
    {
        
        // FlightCtrlService.Takeoff()
        muas::FlightCtrl_Takeoff_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::FlightCtrl_Takeoff_Response parse success");
            auto it = Takeoff_Callbacks.find(RequestID);
            if (it != Takeoff_Callbacks.end())
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
                        Takeoff_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    Takeoff_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::FlightCtrl_Takeoff_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("Land")))
    {
        
        // FlightCtrlService.Land()
        muas::FlightCtrl_Land_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::FlightCtrl_Land_Response parse success");
            auto it = Land_Callbacks.find(RequestID);
            if (it != Land_Callbacks.end())
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
                        Land_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    Land_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::FlightCtrl_Land_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("RTL")))
    {
        
        // FlightCtrlService.RTL()
        muas::FlightCtrl_RTL_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::FlightCtrl_RTL_Response parse success");
            auto it = RTL_Callbacks.find(RequestID);
            if (it != RTL_Callbacks.end())
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
                        RTL_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    RTL_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::FlightCtrl_RTL_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("Kill")))
    {
        
        // FlightCtrlService.Kill()
        muas::FlightCtrl_Kill_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::FlightCtrl_Kill_Response parse success");
            auto it = Kill_Callbacks.find(RequestID);
            if (it != Kill_Callbacks.end())
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
                        Kill_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    Kill_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::FlightCtrl_Kill_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("SetSpeed")))
    {
        
        // FlightCtrlService.SetSpeed()
        muas::FlightCtrl_SetSpeed_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::FlightCtrl_SetSpeed_Response parse success");
            auto it = SetSpeed_Callbacks.find(RequestID);
            if (it != SetSpeed_Callbacks.end())
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
                        SetSpeed_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    SetSpeed_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::FlightCtrl_SetSpeed_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("Reposition")))
    {
        
        // FlightCtrlService.Reposition()
        muas::FlightCtrl_Reposition_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::FlightCtrl_Reposition_Response parse success");
            auto it = Reposition_Callbacks.find(RequestID);
            if (it != Reposition_Callbacks.end())
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
                        Reposition_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    Reposition_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::FlightCtrl_Reposition_Response parse failed");
        }
    }
    
}