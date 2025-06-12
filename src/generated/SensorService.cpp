#include "./SensorService.hpp"

namespace muas
{
    NDN_LOG_INIT(muas.SensorService);

    SensorService::~SensorService() {}

    void SensorService::ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage)
    {
        // log the parameters
        NDN_LOG_INFO("ConsumeRequest: RequesterName: " << RequesterName << " providerName: " << providerName << " ServiceName: " << ServiceName << " FunctionName: " << FunctionName << " RequestID: " << RequestID);
        
        //the payload of the request message is a protobuf message, which is deserialized by the following code:
        ndn::Buffer payload = requestMessage.getPayload();

        
        if (ServiceName.equals(ndn::Name("Sensor")) & FunctionName.equals(ndn::Name("GetSensorInfo")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} GetSensorInfo");
            muas::SensorCtrl_GetSensorInfo_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::SensorCtrl_GetSensorInfo_Request parse success");
                muas::SensorCtrl_GetSensorInfo_Response _response;
                GetSensorInfo(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::SensorCtrl_GetSensorInfo_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Sensor")) & FunctionName.equals(ndn::Name("CaptureSingle")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} CaptureSingle");
            muas::SensorCtrl_CaptureSingle_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::SensorCtrl_CaptureSingle_Request parse success");
                muas::SensorCtrl_CaptureSingle_Response _response;
                CaptureSingle(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::SensorCtrl_CaptureSingle_Request parse failed");
            }
        }
        
        if (ServiceName.equals(ndn::Name("Sensor")) & FunctionName.equals(ndn::Name("CapturePeriodic")))
        {
            NDN_LOG_INFO("OnRequestDecryptionSuccessCallback: {ServiceName} CapturePeriodic");
            muas::SensorCtrl_CapturePeriodic_Request _request;
            if (_request.ParseFromArray(payload.data(), payload.size()))
            {
                NDN_LOG_INFO("onRequestDecryptionSuccessCallback muas::SensorCtrl_CapturePeriodic_Request parse success");
                muas::SensorCtrl_CapturePeriodic_Response _response;
                CapturePeriodic(RequesterName, _request, _response);
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
                NDN_LOG_ERROR("OnRequestDecryptionSuccessCallback muas::SensorCtrl_CapturePeriodic_Request parse failed");
            }
        }
        

    }

    
    void SensorService::GetSensorInfo(const ndn::Name &requesterIdentity, const muas::SensorCtrl_GetSensorInfo_Request &_request, muas::SensorCtrl_GetSensorInfo_Response &_response)
    {
        NDN_LOG_INFO("GetSensorInfo request: " << _request.DebugString());
        // RPC logic starts here
        if (GetSensorInfo_Handler) {
            GetSensorInfo_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No GetSensorInfo handler set.");
        }

        // RPC logic ends here
    }
    
    void SensorService::CaptureSingle(const ndn::Name &requesterIdentity, const muas::SensorCtrl_CaptureSingle_Request &_request, muas::SensorCtrl_CaptureSingle_Response &_response)
    {
        NDN_LOG_INFO("CaptureSingle request: " << _request.DebugString());
        // RPC logic starts here
        if (CaptureSingle_Handler) {
            CaptureSingle_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No CaptureSingle handler set.");
        }

        // RPC logic ends here
    }
    
    void SensorService::CapturePeriodic(const ndn::Name &requesterIdentity, const muas::SensorCtrl_CapturePeriodic_Request &_request, muas::SensorCtrl_CapturePeriodic_Response &_response)
    {
        NDN_LOG_INFO("CapturePeriodic request: " << _request.DebugString());
        // RPC logic starts here
        if (CapturePeriodic_Handler) {
            CapturePeriodic_Handler(requesterIdentity, _request, _response);
        } else {
            NDN_LOG_ERROR("No CapturePeriodic handler set.");
        }

        // RPC logic ends here
    }
    
}