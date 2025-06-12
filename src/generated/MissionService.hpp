#ifndef MissionService_HPP
#define MissionService_HPP

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/face.hpp>

#include "ndn-service-framework/ServiceProvider.hpp"
#include "ndn-service-framework/ServiceStub.hpp"
#include "ndn-service-framework/Service.hpp"

#include "messages.pb.h"

#include <vector>

namespace muas
{

    
    using GetMissionInfo_Function = std::function<void(const ndn::Name &, const muas::Mission_GetMissionInfo_Request &, muas::Mission_GetMissionInfo_Response &)>;
    
    using GetItem_Function = std::function<void(const ndn::Name &, const muas::Mission_GetItem_Request &, muas::Mission_GetItem_Response &)>;
    
    using SetItem_Function = std::function<void(const ndn::Name &, const muas::Mission_SetItem_Request &, muas::Mission_SetItem_Response &)>;
    
    using Clear_Function = std::function<void(const ndn::Name &, const muas::Mission_Clear_Request &, muas::Mission_Clear_Response &)>;
    
    using Start_Function = std::function<void(const ndn::Name &, const muas::Mission_Start_Request &, muas::Mission_Start_Response &)>;
    
    using Pause_Function = std::function<void(const ndn::Name &, const muas::Mission_Pause_Request &, muas::Mission_Pause_Response &)>;
    
    using Continue_Function = std::function<void(const ndn::Name &, const muas::Mission_Continue_Request &, muas::Mission_Continue_Response &)>;
    
    using Terminate_Function = std::function<void(const ndn::Name &, const muas::Mission_Terminate_Request &, muas::Mission_Terminate_Response &)>;
    

    class MissionService : public ndn_service_framework::Service
    {
    public:
        MissionService(ndn_service_framework::ServiceProvider &serviceProvider)
            : ndn_service_framework::Service(serviceProvider),
              serviceName("Mission")
        {
        }

        virtual ~MissionService();

        void ConsumeRequest(const ndn::Name& RequesterName,const ndn::Name& providerName,const ndn::Name& ServiceName,const ndn::Name& FunctionName, const ndn::Name& RequestID, ndn_service_framework::RequestMessage& requestMessage) override;
        
        void GetMissionInfo(const ndn::Name &requesterIdentity, const muas::Mission_GetMissionInfo_Request &_request, muas::Mission_GetMissionInfo_Response &_response);
        
        void GetItem(const ndn::Name &requesterIdentity, const muas::Mission_GetItem_Request &_request, muas::Mission_GetItem_Response &_response);
        
        void SetItem(const ndn::Name &requesterIdentity, const muas::Mission_SetItem_Request &_request, muas::Mission_SetItem_Response &_response);
        
        void Clear(const ndn::Name &requesterIdentity, const muas::Mission_Clear_Request &_request, muas::Mission_Clear_Response &_response);
        
        void Start(const ndn::Name &requesterIdentity, const muas::Mission_Start_Request &_request, muas::Mission_Start_Response &_response);
        
        void Pause(const ndn::Name &requesterIdentity, const muas::Mission_Pause_Request &_request, muas::Mission_Pause_Response &_response);
        
        void Continue(const ndn::Name &requesterIdentity, const muas::Mission_Continue_Request &_request, muas::Mission_Continue_Response &_response);
        
        void Terminate(const ndn::Name &requesterIdentity, const muas::Mission_Terminate_Request &_request, muas::Mission_Terminate_Response &_response);
        


    public:
        ndn::Name serviceName;
        
        GetMissionInfo_Function GetMissionInfo_Handler;
        
        GetItem_Function GetItem_Handler;
        
        SetItem_Function SetItem_Handler;
        
        Clear_Function Clear_Handler;
        
        Start_Function Start_Handler;
        
        Pause_Function Pause_Handler;
        
        Continue_Function Continue_Handler;
        
        Terminate_Function Terminate_Handler;
        
    };
}
#endif