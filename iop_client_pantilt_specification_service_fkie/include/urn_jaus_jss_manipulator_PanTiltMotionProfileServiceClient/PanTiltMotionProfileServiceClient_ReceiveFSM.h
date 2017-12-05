

#ifndef PANTILTMOTIONPROFILESERVICECLIENT_RECEIVEFSM_H
#define PANTILTMOTIONPROFILESERVICECLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_manipulator_PanTiltMotionProfileServiceClient/Messages/MessageSet.h"
#include "urn_jaus_jss_manipulator_PanTiltMotionProfileServiceClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_client_pantilt_specification_service_fkie/PanTiltSpecificationListenerInterface.h>
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/PanTiltSpecificationServiceClientService.h"
#include "PanTiltMotionProfileServiceClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_manipulator_PanTiltMotionProfileServiceClient
{

class DllExport PanTiltMotionProfileServiceClient_ReceiveFSM :
public JTS::StateMachine,
public iop::ocu::SlaveHandlerInterface,
public iop::PanTiltSpecificationListenerInterface
{
public:
	PanTiltMotionProfileServiceClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~PanTiltMotionProfileServiceClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportPanTiltMotionProfileAction(ReportPanTiltMotionProfile msg, Receive::Body::ReceiveRec transportData);

	// PanTiltSpecificationListenerInterface methods
	void pantilt_specification_received(JausAddress reporter, urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::ReportPanTiltSpecifications spec);
	/// Guard Methods

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);

	PanTiltMotionProfileServiceClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::PanTiltSpecificationServiceClientService* pPtSpecService;

	QueryPanTiltMotionProfile p_query_mp;
	JausAddress p_remote_addr;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	bool p_has_access;

	void pQueryCallback(const ros::TimerEvent& event);
};

};

#endif // PANTILTMOTIONPROFILESERVICECLIENT_RECEIVEFSM_H
