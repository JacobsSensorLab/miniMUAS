#include "./WUASServiceStub.hpp"

NDN_LOG_INIT(muas.WUASServiceStub);

muas::WUASServiceStub::WUASServiceStub(ndn::Face& face, ndn_service_framework::ServiceUser &user)
    : ndn_service_framework::ServiceStub(face, user),
      serviceName("WUAS")
{
}

muas::WUASServiceStub::~WUASServiceStub(){}


void muas::WUASServiceStub::QuadRaster_Async(const std::vector<ndn::Name>& providers, const muas::WUAS_QuadRaster_Request &_request, muas::QuadRaster_Callback _callback, muas::QuadRaster_Timeout_Callback _timeout_callback, int timeout_ms, const size_t strategy)
{
    NDN_LOG_INFO("QuadRaster_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::WUAS_QuadRaster_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("WUAS"), ndn::Name("QuadRaster"), requestId, payload, strategy);
    QuadRaster_Callbacks.emplace(requestId, _callback);
    QuadRaster_Timeout_Callbacks.emplace(requestId, _timeout_callback);
    strategyMap.emplace(requestId, strategy);
    
    m_scheduler.schedule(ndn::time::milliseconds(timeout_ms), [this, requestId, _request] { 
        // time out
        this->QuadRaster_Callbacks.erase(requestId);
        // check if timeout_callback is still valid
        auto it = QuadRaster_Timeout_Callbacks.find(requestId);
        if (it != QuadRaster_Timeout_Callbacks.end()) {
            it->second(_request);
        }
    });
}


void muas::WUASServiceStub::OnResponseDecryptionSuccessCallback(const ndn::Name &serviceProviderName, const ndn::Name &ServiceName, const ndn::Name &FunctionName, const ndn::Name &RequestID, const ndn::Buffer &buffer)
{
    NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: " << serviceProviderName << ServiceName << FunctionName << RequestID);

    // parse Response Message from buffer
    ndn_service_framework::ResponseMessage responseMessage;
    responseMessage.WireDecode(ndn::Block(buffer));
    responseMessage.getErrorInfo();

    ndn::Buffer payload = responseMessage.getPayload();

    
    if (ServiceName.equals(ndn::Name("WUAS")) & FunctionName.equals(ndn::Name("QuadRaster")))
    {
        
        // WUASService.QuadRaster()
        muas::WUAS_QuadRaster_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::WUAS_QuadRaster_Response parse success");
            auto it = QuadRaster_Callbacks.find(RequestID);
            if (it != QuadRaster_Callbacks.end())
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
                        QuadRaster_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                    // remove timeout callback if receive any response
                    QuadRaster_Timeout_Callbacks.erase(RequestID);
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::WUAS_QuadRaster_Response parse failed");
        }
    }
    
}