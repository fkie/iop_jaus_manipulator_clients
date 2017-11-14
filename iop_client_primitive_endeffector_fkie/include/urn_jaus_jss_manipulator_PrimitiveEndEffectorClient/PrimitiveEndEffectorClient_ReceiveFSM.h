

#ifndef PRIMITIVEENDEFFECTORCLIENT_RECEIVEFSM_H
#define PRIMITIVEENDEFFECTORCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_manipulator_PrimitiveEndEffectorClient/Messages/MessageSet.h"
#include "urn_jaus_jss_manipulator_PrimitiveEndEffectorClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_events_fkie/EventHandlerInterface.h>

#include "PrimitiveEndEffectorClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_manipulator_PrimitiveEndEffectorClient
{

class DllExport PrimitiveEndEffectorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	PrimitiveEndEffectorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM);
	virtual ~PrimitiveEndEffectorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportEndEffectorEffortAction(ReportEndEffectorEffort msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportEndEffectorSpecificationAction(ReportEndEffectorSpecification msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);



	PrimitiveEndEffectorClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;

	QueryEndEffectorEffort p_query_effort;
	QueryEndEffectorSpecification p_query_spec;

	JausAddress p_manipulator_id;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	ros::Subscriber p_sub_jointstates;
	ros::Subscriber p_sub_cmd_vel;
	ros::Publisher p_pub_jointstates;
	ros::Publisher p_pub_float;
	std::string p_gripper_joint;

	int p_query_state;
	bool p_by_query;
	double p_hz;

	JausAddress p_remote_addr;
	bool p_has_access;
	void pQueryCallback(const ros::TimerEvent& event);
	void pRosCmdJointState(const sensor_msgs::JointState::ConstPtr& joint_state);
	void pRosCmdVelocity(const std_msgs::Float64::ConstPtr& cmd_vel);
	int pGetNameIndexFromJointState(const sensor_msgs::JointState::ConstPtr& joint_state, std::string name);
};

};

#endif // PRIMITIVEENDEFFECTORCLIENT_RECEIVEFSM_H
