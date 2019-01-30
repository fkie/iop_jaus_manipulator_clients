

#ifndef PRIMITIVEPANTILTCLIENT_RECEIVEFSM_H
#define PRIMITIVEPANTILTCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_manipulator_PrimitivePanTiltClient/Messages/MessageSet.h"
#include "urn_jaus_jss_manipulator_PrimitivePanTiltClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>
#include <fkie_iop_client_pantilt_specification_service/PanTiltSpecificationListenerInterface.h>
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/PanTiltSpecificationServiceClientService.h"

#include "PrimitivePanTiltClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_manipulator_PrimitivePanTiltClient
{

class DllExport PrimitivePanTiltClient_ReceiveFSM : public JTS::StateMachine,
														public iop::ocu::SlaveHandlerInterface,
														public iop::PanTiltSpecificationListenerInterface,
														public iop::EventHandlerInterface
{
public:
	static double DEFAUL_MAX_SPEED;
	PrimitivePanTiltClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM);
	virtual ~PrimitivePanTiltClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportPanTiltJointEffortAction(ReportPanTiltJointEffort msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods

	/// SpecificationListenerInterface Methods
	void pantilt_specification_received(JausAddress reporter, urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::ReportPanTiltSpecifications spec);

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);


	PrimitivePanTiltClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;
	urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::PanTiltSpecificationServiceClientService* pPtSpecService;


	QueryPanTiltJointEffort p_query_effort;

	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;
	std::string p_joint1_name;
	std::string p_joint2_name;
	double p_cmd_joint1_vel;
	double p_cmd_joint2_vel;
	double p_joint1_max_vel;
	double p_joint2_max_vel;
	ros::Subscriber p_sub_cmd_vel_joints;
	ros::Subscriber p_sub_cmd_vel_pan;
	ros::Subscriber p_sub_cmd_vel_tilt;
	ros::Subscriber p_sub_cmd_vel_pan32;
	ros::Subscriber p_sub_cmd_vel_tilt32;
	ros::Subscriber p_sub_cmd_twist;

	ros::Publisher p_pub_vel_joints;
	ros::Publisher p_pub_vel_pan;
	ros::Publisher p_pub_vel_tilt;
	ros::Publisher p_pub_vel_pan32;
	ros::Publisher p_pub_vel_tilt32;
	ros::Publisher p_pub_vel_twist;

	JausAddress p_remote_addr;
	bool p_has_access;
	bool p_by_query;
	double p_hz;
	void pQueryCallback(const ros::TimerEvent& event);
	double pNormalize(double value, double value_max);
	void pUpdateCmdVelocity(double pan, double tilt);

	void pJoinStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
	void pPanFloatCallback(const std_msgs::Float64::ConstPtr& msg);
	void pTiltFloatCallback(const std_msgs::Float64::ConstPtr& msg);
	void pPanFloat32Callback(const std_msgs::Float32::ConstPtr& msg);
	void pTiltFloat32Callback(const std_msgs::Float32::ConstPtr& msg);
	void pTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
};

};

#endif // PRIMITIVEPANTILTCLIENT_RECEIVEFSM_H
