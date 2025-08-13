#include "./IUASServiceStub.hpp"

NDN_LOG_INIT(muas.IUASServiceStub);

muas::IUASServiceStub::IUASServiceStub(ndn_service_framework::ServiceUser &user)
    : ndn_service_framework::ServiceStub(user),
      serviceName("IUAS")
{
}

muas::IUASServiceStub::~IUASServiceStub(){}


void muas::IUASServiceStub::PointOrbit_Async(const std::vector<ndn::Name>& providers, const muas::IUAS_PointOrbit_Request &_request, muas::PointOrbit_Callback _callback, muas::PointOrbit_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("PointOrbit_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::IUAS_PointOrbit_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("IUAS"), ndn::Name("PointOrbit"), requestId, payload, strategy);
    PointOrbit_Callbacks.emplace(requestId, _callback);
    PointOrbit_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->PointOrbit_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}

void muas::IUASServiceStub::PointHover_Async(const std::vector<ndn::Name>& providers, const muas::IUAS_PointHover_Request &_request, muas::PointHover_Callback _callback, muas::PointHover_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("PointHover_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::IUAS_PointHover_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("IUAS"), ndn::Name("PointHover"), requestId, payload, strategy);
    PointHover_Callbacks.emplace(requestId, _callback);
    PointHover_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request, _timeout_callback] { 
        // time out
        this->PointHover_Callbacks.erase(requestId);
        _timeout_callback(_request);
    });
}


void muas::IUASServiceStub::OnResponseDecryptionSuccessCallback(const ndn::Name &serviceProviderName, const ndn::Name &ServiceName, const ndn::Name &FunctionName, const ndn::Name &RequestID, const ndn::Buffer &buffer)
{
    NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: " << serviceProviderName << ServiceName << FunctionName << RequestID);

    // parse Response Message from buffer
    ndn_service_framework::ResponseMessage responseMessage;
    responseMessage.WireDecode(ndn::Block(buffer));
    responseMessage.getErrorInfo();

    ndn::Buffer payload = responseMessage.getPayload();

    
    if (ServiceName.equals(ndn::Name("IUAS")) & FunctionName.equals(ndn::Name("PointOrbit")))
    {
        
        // IUASService.PointOrbit()
        muas::IUAS_PointOrbit_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::IUAS_PointOrbit_Response parse success");
            auto it = PointOrbit_Callbacks.find(RequestID);
            if (it != PointOrbit_Callbacks.end())
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
                        PointOrbit_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    PointOrbit_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::IUAS_PointOrbit_Response parse failed");
        }
    }
    
    if (ServiceName.equals(ndn::Name("IUAS")) & FunctionName.equals(ndn::Name("PointHover")))
    {
        
        // IUASService.PointHover()
        muas::IUAS_PointHover_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::IUAS_PointHover_Response parse success");
            auto it = PointHover_Callbacks.find(RequestID);
            if (it != PointHover_Callbacks.end())
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
                        PointHover_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    PointHover_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::IUAS_PointHover_Response parse failed");
        }
    }
    
}