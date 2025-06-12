#include "./MissionServiceStub.hpp"

NDN_LOG_INIT(muas.MissionServiceStub);

muas::MissionServiceStub::MissionServiceStub(ndn_service_framework::ServiceUser &user)
    : ndn_service_framework::ServiceStub(user),
      serviceName("Mission")
{
}

muas::MissionServiceStub::~MissionServiceStub(){}


void muas::MissionServiceStub::GetMissionInfo_Async(const std::vector<ndn::Name>& providers, const muas::Mission_GetMissionInfo_Request &_request, muas::GetMissionInfo_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("GetMissionInfo_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Mission_GetMissionInfo_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Mission"), ndn::Name("GetMissionInfo"), requestId, payload, strategy);
    GetMissionInfo_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}

void muas::MissionServiceStub::GetItem_Async(const std::vector<ndn::Name>& providers, const muas::Mission_GetItem_Request &_request, muas::GetItem_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("GetItem_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Mission_GetItem_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Mission"), ndn::Name("GetItem"), requestId, payload, strategy);
    GetItem_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}

void muas::MissionServiceStub::SetItem_Async(const std::vector<ndn::Name>& providers, const muas::Mission_SetItem_Request &_request, muas::SetItem_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("SetItem_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Mission_SetItem_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Mission"), ndn::Name("SetItem"), requestId, payload, strategy);
    SetItem_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}

void muas::MissionServiceStub::Clear_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Clear_Request &_request, muas::Clear_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("Clear_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Mission_Clear_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Mission"), ndn::Name("Clear"), requestId, payload, strategy);
    Clear_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}

void muas::MissionServiceStub::Start_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Start_Request &_request, muas::Start_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("Start_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Mission_Start_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Mission"), ndn::Name("Start"), requestId, payload, strategy);
    Start_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}

void muas::MissionServiceStub::Pause_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Pause_Request &_request, muas::Pause_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("Pause_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Mission_Pause_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Mission"), ndn::Name("Pause"), requestId, payload, strategy);
    Pause_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}

void muas::MissionServiceStub::Continue_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Continue_Request &_request, muas::Continue_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("Continue_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Mission_Continue_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Mission"), ndn::Name("Continue"), requestId, payload, strategy);
    Continue_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}

void muas::MissionServiceStub::Terminate_Async(const std::vector<ndn::Name>& providers, const muas::Mission_Terminate_Request &_request, muas::Terminate_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("Terminate_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::Mission_Terminate_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("Mission"), ndn::Name("Terminate"), requestId, payload, strategy);
    Terminate_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}


void muas::MissionServiceStub::OnResponseDecryptionSuccessCallback(const ndn::Name &serviceProviderName, const ndn::Name &ServiceName, const ndn::Name &FunctionName, const ndn::Name &RequestID, const ndn::Buffer &buffer)
{
    NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: " << serviceProviderName << ServiceName << FunctionName << RequestID);

    // parse Response Message from buffer
    ndn_service_framework::ResponseMessage responseMessage;
    responseMessage.WireDecode(ndn::Block(buffer));
    responseMessage.getErrorInfo();

    ndn::Buffer payload = responseMessage.getPayload();

    
    if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("GetMissionInfo")))
    {
        
        // MissionService.GetMissionInfo()
        muas::Mission_GetMissionInfo_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Mission_GetMissionInfo_Response parse success");
            auto it = GetMissionInfo_Callbacks.find(RequestID);
            if (it != GetMissionInfo_Callbacks.end())
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
                        GetMissionInfo_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Mission_GetMissionInfo_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("GetItem")))
    {
        
        // MissionService.GetItem()
        muas::Mission_GetItem_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Mission_GetItem_Response parse success");
            auto it = GetItem_Callbacks.find(RequestID);
            if (it != GetItem_Callbacks.end())
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
                        GetItem_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Mission_GetItem_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("SetItem")))
    {
        
        // MissionService.SetItem()
        muas::Mission_SetItem_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Mission_SetItem_Response parse success");
            auto it = SetItem_Callbacks.find(RequestID);
            if (it != SetItem_Callbacks.end())
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
                        SetItem_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Mission_SetItem_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Clear")))
    {
        
        // MissionService.Clear()
        muas::Mission_Clear_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Mission_Clear_Response parse success");
            auto it = Clear_Callbacks.find(RequestID);
            if (it != Clear_Callbacks.end())
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
                        Clear_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Mission_Clear_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Start")))
    {
        
        // MissionService.Start()
        muas::Mission_Start_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Mission_Start_Response parse success");
            auto it = Start_Callbacks.find(RequestID);
            if (it != Start_Callbacks.end())
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
                        Start_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Mission_Start_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Pause")))
    {
        
        // MissionService.Pause()
        muas::Mission_Pause_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Mission_Pause_Response parse success");
            auto it = Pause_Callbacks.find(RequestID);
            if (it != Pause_Callbacks.end())
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
                        Pause_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Mission_Pause_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Continue")))
    {
        
        // MissionService.Continue()
        muas::Mission_Continue_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Mission_Continue_Response parse success");
            auto it = Continue_Callbacks.find(RequestID);
            if (it != Continue_Callbacks.end())
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
                        Continue_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Mission_Continue_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Terminate")))
    {
        
        // MissionService.Terminate()
        muas::Mission_Terminate_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::Mission_Terminate_Response parse success");
            auto it = Terminate_Callbacks.find(RequestID);
            if (it != Terminate_Callbacks.end())
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
                        Terminate_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::Mission_Terminate_Response parse failed");
        }
    }
    
}