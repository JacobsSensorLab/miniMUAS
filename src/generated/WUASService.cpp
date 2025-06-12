#include "./WUASService.hpp"

namespace muas
{
    NDN_LOG_INIT(muas.WUASService);

    WUASService::~WUASService() {}

    void WUASService::ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage)
    {
        // log the parameters
        NDN_LOG_INFO("ConsumeRequest: RequesterName: " << RequesterName << " providerName: " << providerName << " ServiceName: " << ServiceName << " FunctionName: " << FunctionName << " RequestID: " << RequestID);
        
        //the payload of the request message is a protobuf message, which is deserialized by the following code:
        ndn::Buffer payload = requestMessage.getPayload();

        
        if (ServiceName.equals(ndn::Name("WUAS")) & FunctionName.equals(ndn::Name("QuadRaster")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} QuadRaster");
            muas::WUAS_QuadRaster_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::WUAS_QuadRaster_Request parse success");
                muas::WUAS_QuadRaster_Response _response;
                QuadRaster(RequesterName, _request, _response);
                std::string buffer = "";
                _response.SerializeToString(&buffer);
                ndn::Buffer resPayload(reinterpret_cast<const uint8_t *>(buffer.data()), buffer.size());
                // make ResponseMessage and publish it
                ndn_service_framework::ResponseMessage responseMessage;
                responseMessage.setStatus(true);
                responseMessage.setErrorInfo("No error");
                responseMessage.setPayload(const_cast<ndn::Buffer&>(resPayload), resPayload.size());

                // make response name and response name without prefix
                ndn::Name responseName = ndn_service_framework::makeResponseName(providerName, RequesterName, ServiceName, FunctionName, RequestID);
                ndn::Name responseNameWithoutPrefix = ndn_service_framework::makeResponseNameWithoutPrefix(RequesterName, ServiceName, FunctionName, RequestID);
                m_ServiceProvider->PublishMessage(responseName, responseNameWithoutPrefix, responseMessage);
            }
            else
            {
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::WUAS_QuadRaster_Request parse failed");
            }
        }
        

    }

    
    void WUASService::QuadRaster(const ndn::Name &requesterIdentity, const muas::WUAS_QuadRaster_Request &_request, muas::WUAS_QuadRaster_Response &_response)
    {
        NDN_LOG_INFO("QuadRaster request: " << _request.DebugString());
        // RPC logic starts here
        if (QuadRaster_Handler) {
            QuadRaster_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No QuadRaster handler set.");
        }

        // RPC logic ends here
    }
    
}