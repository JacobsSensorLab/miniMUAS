#include "./MAVLinkServiceStub.hpp"

NDN_LOG_INIT(muas.MAVLinkServiceStub);

muas::MAVLinkServiceStub::MAVLinkServiceStub(ndn_service_framework::ServiceUser &user)
    : ndn_service_framework::ServiceStub(user),
      serviceName("MAVLink")
{
}

muas::MAVLinkServiceStub::~MAVLinkServiceStub(){}


void muas::MAVLinkServiceStub::Generic_Async(const std::vector<ndn::Name>& providers, const muas::MAVLink_Generic_Request &_request, muas::Generic_Callback _callback,  const size_t strategy)
{
    NDN_LOG_INFO("Generic_Async "<<"provider:"<<providers.size()<<" request:"<<_request.DebugString());
    muas::MAVLink_Generic_Response response;
    std::string buffer = "";
    _request.SerializeToString(&buffer);
    ndn::Buffer payload(buffer.begin(),buffer.end());
    ndn::Name requestId(ndn::time::toIsoString(ndn::time::system_clock::now()));
    m_user->PublishRequest(providers, ndn::Name("MAVLink"), ndn::Name("Generic"), requestId, payload, strategy);
    Generic_Callbacks.emplace(requestId, _callback);
    strategyMap.emplace(requestId, strategy);
}


void muas::MAVLinkServiceStub::OnResponseDecryptionSuccessCallback(const ndn::Name &serviceProviderName, const ndn::Name &ServiceName, const ndn::Name &FunctionName, const ndn::Name &RequestID, const ndn::Buffer &buffer)
{
    NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: " << serviceProviderName << ServiceName << FunctionName << RequestID);

    // parse Response Message from buffer
    ndn_service_framework::ResponseMessage responseMessage;
    responseMessage.WireDecode(ndn::Block(buffer));
    responseMessage.getErrorInfo();

    ndn::Buffer payload = responseMessage.getPayload();

    
    if (ServiceName.equals(ndn::Name("MAVLink")) & FunctionName.equals(ndn::Name("Generic")))
    {
        
        // MAVLinkService.Generic()
        muas::MAVLink_Generic_Response _response;
        if (_response.ParseFromArray(payload.data(), payload.size()))
        {
            NDN_LOG_INFO("OnResponseDecryptionSuccessCallback muas::MAVLink_Generic_Response parse success");
            auto it = Generic_Callbacks.find(RequestID);
            if (it != Generic_Callbacks.end())
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
                        Generic_Callbacks.erase(it);
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Remove used callback");
                    }else{
                        NDN_LOG_INFO("OnResponseDecryptionSuccessCallback: Keep callback for ndn_service_framework::tlv::NoCoordination");
                    }
                }
            }
        }
        else
        {
            NDN_LOG_ERROR("OnResponseDecryptionSuccessCallback muas::MAVLink_Generic_Response parse failed");
        }
    }
    
}