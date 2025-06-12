#include "./EntityService.hpp"

namespace muas
{
    NDN_LOG_INIT(muas.EntityService);

    EntityService::~EntityService() {}

    void EntityService::ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage)
    {
        // log the parameters
        NDN_LOG_INFO("ConsumeRequest: RequesterName: " << RequesterName << " providerName: " << providerName << " ServiceName: " << ServiceName << " FunctionName: " << FunctionName << " RequestID: " << RequestID);
        
        //the payload of the request message is a protobuf message, which is deserialized by the following code:
        ndn::Buffer payload = requestMessage.getPayload();

        
        if (ServiceName.equals(ndn::Name("Entity")) & FunctionName.equals(ndn::Name("Echo")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Echo");
            muas::Entity_Echo_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Entity_Echo_Request parse success");
                muas::Entity_Echo_Response _response;
                Echo(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Entity_Echo_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Entity")) & FunctionName.equals(ndn::Name("GetEntityInfo")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} GetEntityInfo");
            muas::Entity_GetEntityInfo_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Entity_GetEntityInfo_Request parse success");
                muas::Entity_GetEntityInfo_Response _response;
                GetEntityInfo(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Entity_GetEntityInfo_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Entity")) & FunctionName.equals(ndn::Name("GetPosition")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} GetPosition");
            muas::Entity_GetPosition_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Entity_GetPosition_Request parse success");
                muas::Entity_GetPosition_Response _response;
                GetPosition(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Entity_GetPosition_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Entity")) & FunctionName.equals(ndn::Name("GetOrientation")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} GetOrientation");
            muas::Entity_GetOrientation_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Entity_GetOrientation_Request parse success");
                muas::Entity_GetOrientation_Response _response;
                GetOrientation(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Entity_GetOrientation_Request parse failed");
            }
        }
        

    }

    
    void EntityService::Echo(const ndn::Name &requesterIdentity, const muas::Entity_Echo_Request &_request, muas::Entity_Echo_Response &_response)
    {
        NDN_LOG_INFO("Echo request: " << _request.DebugString());
        // RPC logic starts here
        if (Echo_Handler) {
            Echo_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Echo handler set.");
        }

        // RPC logic ends here
    }
    
    void EntityService::GetEntityInfo(const ndn::Name &requesterIdentity, const muas::Entity_GetEntityInfo_Request &_request, muas::Entity_GetEntityInfo_Response &_response)
    {
        NDN_LOG_INFO("GetEntityInfo request: " << _request.DebugString());
        // RPC logic starts here
        if (GetEntityInfo_Handler) {
            GetEntityInfo_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No GetEntityInfo handler set.");
        }

        // RPC logic ends here
    }
    
    void EntityService::GetPosition(const ndn::Name &requesterIdentity, const muas::Entity_GetPosition_Request &_request, muas::Entity_GetPosition_Response &_response)
    {
        NDN_LOG_INFO("GetPosition request: " << _request.DebugString());
        // RPC logic starts here
        if (GetPosition_Handler) {
            GetPosition_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No GetPosition handler set.");
        }

        // RPC logic ends here
    }
    
    void EntityService::GetOrientation(const ndn::Name &requesterIdentity, const muas::Entity_GetOrientation_Request &_request, muas::Entity_GetOrientation_Response &_response)
    {
        NDN_LOG_INFO("GetOrientation request: " << _request.DebugString());
        // RPC logic starts here
        if (GetOrientation_Handler) {
            GetOrientation_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No GetOrientation handler set.");
        }

        // RPC logic ends here
    }
    
}