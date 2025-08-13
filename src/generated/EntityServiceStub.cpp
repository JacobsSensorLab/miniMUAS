#include "./EntityServiceStub.hpp"

NDN_LOG_INIT(muas.EntityServiceStub);

muas::EntityServiceStub::EntityServiceStub(ndn_service_framework::ServiceUser &user)
    : ndn_service_framework::ServiceStub(user),
      serviceName("Entity")
{
}

muas::EntityServiceStub::~EntityServiceStub(){}


void muas::EntityServiceStub::Echo_Async(const std::vector<ndn::Name>& providers, const muas::Entity_Echo_Request &_request, muas::Echo_Callback _callback, muas::Echo_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("Echo_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Entity_Echo_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Entity"), ndn::Name("Echo"), requestId, payload, strategy);
    Echo_Callbacks.emplace(requestId, _callback);
    Echo_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->Echo_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}

void muas::EntityServiceStub::GetEntityInfo_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetEntityInfo_Request &_request, muas::GetEntityInfo_Callback _callback, muas::GetEntityInfo_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("GetEntityInfo_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Entity_GetEntityInfo_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Entity"), ndn::Name("GetEntityInfo"), requestId, payload, strategy);
    GetEntityInfo_Callbacks.emplace(requestId, _callback);
    GetEntityInfo_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->GetEntityInfo_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}

void muas::EntityServiceStub::GetPosition_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetPosition_Request &_request, muas::GetPosition_Callback _callback, muas::GetPosition_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("GetPosition_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Entity_GetPosition_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Entity"), ndn::Name("GetPosition"), requestId, payload, strategy);
    GetPosition_Callbacks.emplace(requestId, _callback);
    GetPosition_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->GetPosition_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}

void muas::EntityServiceStub::GetOrientation_Async(const std::vector<ndn::Name>& providers, const muas::Entity_GetOrientation_Request &_request, muas::GetOrientation_Callback _callback, muas::GetOrientation_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("GetOrientation_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Entity_GetOrientation_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Entity"), ndn::Name("GetOrientation"), requestId, payload, strategy);
    GetOrientation_Callbacks.emplace(requestId, _callback);
    GetOrientation_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->GetOrientation_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}


void muas::EntityServiceStub::OnResponseDecryptionSuccessCallback(const ndn::Name &serviceProviderName, const ndn::Name &ServiceName, const ndn::Name &FunctionName, const ndn::Name &RequestID, const ndn::Buffer &buffer)
{
    NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: " << serviceProviderName << ServiceName << FunctionName << RequestID);

    // parse Response Message from buffer
    ndn_service_framework::ResponseMessage responseMessage;
    responseMessage.WireDecode(ndn::Block(buffer));
    responseMessage.getErrorInfo();

    ndn::Buffer payload = responseMessage.getPayload();

    
    if (ServiceName.equals(ndn::Name("Entity")) & FunctionName.equals(ndn::Name("Echo")))
    {
        
        // EntityService.Echo()
        muas::Entity_Echo_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Entity_Echo_Response parse success");
            auto it = Echo_Callbacks.find(RequestID);
            if (it != Echo_Callbacks.end())
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
                        Echo_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    Echo_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Entity_Echo_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Entity")) & FunctionName.equals(ndn::Name("GetEntityInfo")))
    {
        
        // EntityService.GetEntityInfo()
        muas::Entity_GetEntityInfo_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Entity_GetEntityInfo_Response parse success");
            auto it = GetEntityInfo_Callbacks.find(RequestID);
            if (it != GetEntityInfo_Callbacks.end())
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
                        GetEntityInfo_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    GetEntityInfo_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Entity_GetEntityInfo_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Entity")) & FunctionName.equals(ndn::Name("GetPosition")))
    {
        
        // EntityService.GetPosition()
        muas::Entity_GetPosition_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Entity_GetPosition_Response parse success");
            auto it = GetPosition_Callbacks.find(RequestID);
            if (it != GetPosition_Callbacks.end())
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
                        GetPosition_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    GetPosition_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Entity_GetPosition_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Entity")) & FunctionName.equals(ndn::Name("GetOrientation")))
    {
        
        // EntityService.GetOrientation()
        muas::Entity_GetOrientation_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Entity_GetOrientation_Response parse success");
            auto it = GetOrientation_Callbacks.find(RequestID);
            if (it != GetOrientation_Callbacks.end())
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
                        GetOrientation_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    GetOrientation_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Entity_GetOrientation_Response parse failed");
        }
    }
    
}