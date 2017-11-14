

#ifndef MANIPULATORJOINTPOSITIONSENSORCLIENT_RECEIVEFSM_H
#define MANIPULATORJOINTPOSITIONSENSORCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_manipulator_ManipulatorJointPositionSensorClient/Messages/MessageSet.h"
#include "urn_jaus_jss_manipulator_ManipulatorJointPositionSensorClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_events_fkie/EventHandlerInterface.h>
#include <iop_client_manipulator_specification_fkie/SpecificationListenerInterface.h>
#include "urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient/ManipulatorSpecificationServiceClientService.h"


#include "ManipulatorJointPositionSensorClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_manipulator_ManipulatorJointPositionSensorClient
{

class DllExport ManipulatorJointPositionSensorClient_ReceiveFSM :	public JTS::StateMachine,
																	public iop::ocu::SlaveHandlerInterface,
																	public iop::SpecificationListenerInterface,
																	public iop::EventHandlerInterface
{
public:
	ManipulatorJointPositionSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM);
	virtual ~ManipulatorJointPositionSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportJointPositionsAction(ReportJointPositions msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	/// SpecificationListenerInterface Methods
	void specification_received(JausAddress reporter, urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient::ReportManipulatorSpecifications spec);

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);

	ManipulatorJointPositionSensorClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient::ManipulatorSpecificationServiceClientService* pManiSpecService;

	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	ros::Publisher p_pub_jointstates;
	ros::Publisher p_pub_float;
	std::vector<std::string> p_joint_names;
	std::vector<bool> p_joint_types; // True: prismatic joints, False: revolute joints

	QueryJointPositions p_query_joints;
	int p_query_state;
	bool p_by_query;
	double p_hz;

	JausAddress p_remote_addr;
	bool p_has_access;
	void pQueryCallback(const ros::TimerEvent& event);
	int pGetNameIndexFromJointState(const sensor_msgs::JointState::ConstPtr& joint_state, std::string name);

};

};

#endif // MANIPULATORJOINTPOSITIONSENSORCLIENT_RECEIVEFSM_H
