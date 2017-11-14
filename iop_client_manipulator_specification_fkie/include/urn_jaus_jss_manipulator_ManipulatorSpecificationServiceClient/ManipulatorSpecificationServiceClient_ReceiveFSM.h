

#ifndef MANIPULATORSPECIFICATIONSERVICECLIENT_RECEIVEFSM_H
#define MANIPULATORSPECIFICATIONSERVICECLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient/Messages/MessageSet.h"
#include "urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"

#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_events_fkie/EventHandlerInterface.h>
#include <iop_client_manipulator_specification_fkie/SpecificationListenerInterface.h>

#include "ManipulatorSpecificationServiceClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient
{

class DllExport ManipulatorSpecificationServiceClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	ManipulatorSpecificationServiceClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM);
	virtual ~ManipulatorSpecificationServiceClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportManipulatorSpecificationsAction(ReportManipulatorSpecifications msg, Receive::Body::ReceiveRec transportData);


	void add_listener(iop::SpecificationListenerInterface *listener);
	std::vector<std::string> get_joint_names(ReportManipulatorSpecifications spec);
	/** Returns the list with types of joints. True for prismatic joints, False for revolute joints. */
	std::vector<bool> get_joint_types(ReportManipulatorSpecifications spec);

	/// Guard Methods
	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);


	ManipulatorSpecificationServiceClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;

	std::vector<iop::SpecificationListenerInterface*> p_spec_listener;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	ros::Publisher p_pub_navsatfix;
	ros::Publisher p_pub_imu;
	double p_hz;

	QueryManipulatorSpecifications p_query_spec;
	JausAddress p_remote_addr;
	bool p_has_access;
	void pQueryCallback(const ros::TimerEvent& event);
	void pNotifyListeners(JausAddress reporter, ReportManipulatorSpecifications spec);

};

};

#endif // MANIPULATORSPECIFICATIONSERVICECLIENT_RECEIVEFSM_H
