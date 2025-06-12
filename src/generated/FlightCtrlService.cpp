#include "./FlightCtrlService.hpp"

namespace muas
{
    NDN_LOG_INIT(muas.FlightCtrlService);

    FlightCtrlService::~FlightCtrlService() {}

    void FlightCtrlService::ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage)
    {
        // log the parameters
        NDN_LOG_INFO("ConsumeRequest: RequesterName: " << RequesterName << " providerName: " << providerName << " ServiceName: " << ServiceName << " FunctionName: " << FunctionName << " RequestID: " << RequestID);
        
        //the payload of the request message is a protobuf message, which is deserialized by the following code:
        ndn::Buffer payload = requestMessage.getPayload();

        
        if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("SwitchMode")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} SwitchMode");
            muas::FlightCtrl_SwitchMode_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::FlightCtrl_SwitchMode_Request parse success");
                muas::FlightCtrl_SwitchMode_Response _response;
                SwitchMode(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::FlightCtrl_SwitchMode_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("Takeoff")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Takeoff");
            muas::FlightCtrl_Takeoff_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::FlightCtrl_Takeoff_Request parse success");
                muas::FlightCtrl_Takeoff_Response _response;
                Takeoff(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::FlightCtrl_Takeoff_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("Land")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Land");
            muas::FlightCtrl_Land_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::FlightCtrl_Land_Request parse success");
                muas::FlightCtrl_Land_Response _response;
                Land(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::FlightCtrl_Land_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("RTL")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} RTL");
            muas::FlightCtrl_RTL_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::FlightCtrl_RTL_Request parse success");
                muas::FlightCtrl_RTL_Response _response;
                RTL(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::FlightCtrl_RTL_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("Kill")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Kill");
            muas::FlightCtrl_Kill_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::FlightCtrl_Kill_Request parse success");
                muas::FlightCtrl_Kill_Response _response;
                Kill(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::FlightCtrl_Kill_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("SetSpeed")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} SetSpeed");
            muas::FlightCtrl_SetSpeed_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::FlightCtrl_SetSpeed_Request parse success");
                muas::FlightCtrl_SetSpeed_Response _response;
                SetSpeed(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::FlightCtrl_SetSpeed_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("FlightCtrl")) & FunctionName.equals(ndn::Name("Reposition")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} Reposition");
            muas::FlightCtrl_Reposition_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::FlightCtrl_Reposition_Request parse success");
                muas::FlightCtrl_Reposition_Response _response;
                Reposition(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::FlightCtrl_Reposition_Request parse failed");
            }
        }
        

    }

    
    void FlightCtrlService::SwitchMode(const ndn::Name &requesterIdentity, const muas::FlightCtrl_SwitchMode_Request &_request, muas::FlightCtrl_SwitchMode_Response &_response)
    {
        NDN_LOG_INFO("SwitchMode request: " << _request.DebugString());
        // RPC logic starts here
        if (SwitchMode_Handler) {
            SwitchMode_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No SwitchMode handler set.");
        }

        // RPC logic ends here
    }
    
    void FlightCtrlService::Takeoff(const ndn::Name &requesterIdentity, const muas::FlightCtrl_Takeoff_Request &_request, muas::FlightCtrl_Takeoff_Response &_response)
    {
        NDN_LOG_INFO("Takeoff request: " << _request.DebugString());
        // RPC logic starts here
        if (Takeoff_Handler) {
            Takeoff_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Takeoff handler set.");
        }

        // RPC logic ends here
    }
    
    void FlightCtrlService::Land(const ndn::Name &requesterIdentity, const muas::FlightCtrl_Land_Request &_request, muas::FlightCtrl_Land_Response &_response)
    {
        NDN_LOG_INFO("Land request: " << _request.DebugString());
        // RPC logic starts here
        if (Land_Handler) {
            Land_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Land handler set.");
        }

        // RPC logic ends here
    }
    
    void FlightCtrlService::RTL(const ndn::Name &requesterIdentity, const muas::FlightCtrl_RTL_Request &_request, muas::FlightCtrl_RTL_Response &_response)
    {
        NDN_LOG_INFO("RTL request: " << _request.DebugString());
        // RPC logic starts here
        if (RTL_Handler) {
            RTL_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No RTL handler set.");
        }

        // RPC logic ends here
    }
    
    void FlightCtrlService::Kill(const ndn::Name &requesterIdentity, const muas::FlightCtrl_Kill_Request &_request, muas::FlightCtrl_Kill_Response &_response)
    {
        NDN_LOG_INFO("Kill request: " << _request.DebugString());
        // RPC logic starts here
        if (Kill_Handler) {
            Kill_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Kill handler set.");
        }

        // RPC logic ends here
    }
    
    void FlightCtrlService::SetSpeed(const ndn::Name &requesterIdentity, const muas::FlightCtrl_SetSpeed_Request &_request, muas::FlightCtrl_SetSpeed_Response &_response)
    {
        NDN_LOG_INFO("SetSpeed request: " << _request.DebugString());
        // RPC logic starts here
        if (SetSpeed_Handler) {
            SetSpeed_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No SetSpeed handler set.");
        }

        // RPC logic ends here
    }
    
    void FlightCtrlService::Reposition(const ndn::Name &requesterIdentity, const muas::FlightCtrl_Reposition_Request &_request, muas::FlightCtrl_Reposition_Response &_response)
    {
        NDN_LOG_INFO("Reposition request: " << _request.DebugString());
        // RPC logic starts here
        if (Reposition_Handler) {
            Reposition_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No Reposition handler set.");
        }

        // RPC logic ends here
    }
    
}