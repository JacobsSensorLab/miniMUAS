#include "./IUASService.hpp"

namespace muas
{
    NDN_LOG_INIT(muas.IUASService);

    IUASService::~IUASService() {}

    void IUASService::ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage)
    {
        // log the parameters
        NDN_LOG_INFO("ConsumeRequest: RequesterName: " << RequesterName << " providerName: " << providerName << " ServiceName: " << ServiceName << " FunctionName: " << FunctionName << " RequestID: " << RequestID);
        
        //the payload of the request message is a protobuf message, which is deserialized by the following code:
        ndn::Buffer payload = requestMessage.getPayload();

        
        if (ServiceName.equals(ndn::Name("IUAS")) & FunctionName.equals(ndn::Name("PointOrbit")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} PointOrbit");
            muas::IUAS_PointOrbit_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::IUAS_PointOrbit_Request parse success");
                muas::IUAS_PointOrbit_Response _response;
                PointOrbit(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::IUAS_PointOrbit_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("IUAS")) & FunctionName.equals(ndn::Name("PointHover")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} PointHover");
            muas::IUAS_PointHover_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::IUAS_PointHover_Request parse success");
                muas::IUAS_PointHover_Response _response;
                PointHover(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::IUAS_PointHover_Request parse failed");
            }
        }
        

    }

    
    void IUASService::PointOrbit(const ndn::Name &requesterIdentity, const muas::IUAS_PointOrbit_Request &_request, muas::IUAS_PointOrbit_Response &_response)
    {
        NDN_LOG_INFO("PointOrbit request: " << _request.DebugString());
        // RPC logic starts here
        if (PointOrbit_Handler) {
            PointOrbit_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No PointOrbit handler set.");
        }

        // RPC logic ends here
    }
    
    void IUASService::PointHover(const ndn::Name &requesterIdentity, const muas::IUAS_PointHover_Request &_request, muas::IUAS_PointHover_Response &_response)
    {
        NDN_LOG_INFO("PointHover request: " << _request.DebugString());
        // RPC logic starts here
        if (PointHover_Handler) {
            PointHover_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No PointHover handler set.");
        }

        // RPC logic ends here
    }
    
}