#include "./MissionService.hpp"

namespace muas
{
    NDN_LOG_INIT(muas.MissionService);

    MissionService::~MissionService() {}

    void MissionService::ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage)
    {
        // log the parameters
        NDN_LOG_INFO("ConsumeRequest: RequesterName: " << RequesterName << " providerName: " << providerName << " ServiceName: " << ServiceName << " FunctionName: " << FunctionName << " RequestID: " << RequestID);
        
        //the payload of the request message is a protobuf message, which is deserialized by the following code:
        ndn::Buffer payload = requestMessage.getPayload();

        
        if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("GetMissionInfo")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} GetMissionInfo");
            muas::Mission_GetMissionInfo_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Mission_GetMissionInfo_Request parse success");
                muas::Mission_GetMissionInfo_Response _response;
                GetMissionInfo(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Mission_GetMissionInfo_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("GetItem")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} GetItem");
            muas::Mission_GetItem_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Mission_GetItem_Request parse success");
                muas::Mission_GetItem_Response _response;
                GetItem(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Mission_GetItem_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("SetItem")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} SetItem");
            muas::Mission_SetItem_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Mission_SetItem_Request parse success");
                muas::Mission_SetItem_Response _response;
                SetItem(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Mission_SetItem_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Clear")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Clear");
            muas::Mission_Clear_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Mission_Clear_Request parse success");
                muas::Mission_Clear_Response _response;
                Clear(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Mission_Clear_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Start")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Start");
            muas::Mission_Start_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Mission_Start_Request parse success");
                muas::Mission_Start_Response _response;
                Start(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Mission_Start_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Pause")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Pause");
            muas::Mission_Pause_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Mission_Pause_Request parse success");
                muas::Mission_Pause_Response _response;
                Pause(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Mission_Pause_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Continue")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Continue");
            muas::Mission_Continue_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Mission_Continue_Request parse success");
                muas::Mission_Continue_Response _response;
                Continue(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Mission_Continue_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Mission")) & FunctionName.equals(ndn::Name("Terminate")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Terminate");
            muas::Mission_Terminate_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::Mission_Terminate_Request parse success");
                muas::Mission_Terminate_Response _response;
                Terminate(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::Mission_Terminate_Request parse failed");
            }
        }
        

    }

    
    void MissionService::GetMissionInfo(const ndn::Name &requesterIdentity, const muas::Mission_GetMissionInfo_Request &_request, muas::Mission_GetMissionInfo_Response &_response)
    {
        NDN_LOG_INFO("GetMissionInfo request: " << _request.DebugString());
        // RPC logic starts here
        if (GetMissionInfo_Handler) {
            GetMissionInfo_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No GetMissionInfo handler set.");
        }

        // RPC logic ends here
    }
    
    void MissionService::GetItem(const ndn::Name &requesterIdentity, const muas::Mission_GetItem_Request &_request, muas::Mission_GetItem_Response &_response)
    {
        NDN_LOG_INFO("GetItem request: " << _request.DebugString());
        // RPC logic starts here
        if (GetItem_Handler) {
            GetItem_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No GetItem handler set.");
        }

        // RPC logic ends here
    }
    
    void MissionService::SetItem(const ndn::Name &requesterIdentity, const muas::Mission_SetItem_Request &_request, muas::Mission_SetItem_Response &_response)
    {
        NDN_LOG_INFO("SetItem request: " << _request.DebugString());
        // RPC logic starts here
        if (SetItem_Handler) {
            SetItem_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No SetItem handler set.");
        }

        // RPC logic ends here
    }
    
    void MissionService::Clear(const ndn::Name &requesterIdentity, const muas::Mission_Clear_Request &_request, muas::Mission_Clear_Response &_response)
    {
        NDN_LOG_INFO("Clear request: " << _request.DebugString());
        // RPC logic starts here
        if (Clear_Handler) {
            Clear_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Clear handler set.");
        }

        // RPC logic ends here
    }
    
    void MissionService::Start(const ndn::Name &requesterIdentity, const muas::Mission_Start_Request &_request, muas::Mission_Start_Response &_response)
    {
        NDN_LOG_INFO("Start request: " << _request.DebugString());
        // RPC logic starts here
        if (Start_Handler) {
            Start_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Start handler set.");
        }

        // RPC logic ends here
    }
    
    void MissionService::Pause(const ndn::Name &requesterIdentity, const muas::Mission_Pause_Request &_request, muas::Mission_Pause_Response &_response)
    {
        NDN_LOG_INFO("Pause request: " << _request.DebugString());
        // RPC logic starts here
        if (Pause_Handler) {
            Pause_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Pause handler set.");
        }

        // RPC logic ends here
    }
    
    void MissionService::Continue(const ndn::Name &requesterIdentity, const muas::Mission_Continue_Request &_request, muas::Mission_Continue_Response &_response)
    {
        NDN_LOG_INFO("Continue request: " << _request.DebugString());
        // RPC logic starts here
        if (Continue_Handler) {
            Continue_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Continue handler set.");
        }

        // RPC logic ends here
    }
    
    void MissionService::Terminate(const ndn::Name &requesterIdentity, const muas::Mission_Terminate_Request &_request, muas::Mission_Terminate_Response &_response)
    {
        NDN_LOG_INFO("Terminate request: " << _request.DebugString());
        // RPC logic starts here
        if (Terminate_Handler) {
            Terminate_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Terminate handler set.");
        }

        // RPC logic ends here
    }
    
}