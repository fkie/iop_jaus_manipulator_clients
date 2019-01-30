

#ifndef PANTILTJOINTPOSITIONDRIVERCLIENT_RECEIVEFSM_H
#define PANTILTJOINTPOSITIONDRIVERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_manipulator_PanTiltJointPositionDriverClient/Messages/MessageSet.h"
#include "urn_jaus_jss_manipulator_PanTiltJointPositionDriverClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <boost/thread/recursive_mutex.hpp>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

#include <fkie_iop_client_pantilt_specification_service/PanTiltSpecificationListenerInterface.h>
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/PanTiltSpecificationServiceClientService.h"

#include "PanTiltJointPositionDriverClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_manipulator_PanTiltJointPositionDriverClient
{

class DllExport PanTiltJointPositionDriverClient_ReceiveFSM :
public JTS::StateMachine,
public iop::ocu::SlaveHandlerInterface,
public iop::EventHandlerInterface,
public iop::PanTiltSpecificationListenerInterface
{
public:
	PanTiltJointPositionDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM);
	virtual ~PanTiltJointPositionDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportCommandedPanTiltJointPositionsAction(ReportCommandedPanTiltJointPositions msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	// PanTiltSpecificationListenerInterface methods
	void pantilt_specification_received(JausAddress reporter, urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::ReportPanTiltSpecifications spec);

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);


	PanTiltJointPositionDriverClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;
	urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::PanTiltSpecificationServiceClientService* pPtSpecService;

	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;
	std::string p_joint1_name;
	std::string p_joint2_name;
	double p_cmd_joint1_position;
	double p_cmd_joint2_position;
	ros::Subscriber p_sub_cmd_pos_joints;
	ros::Subscriber p_sub_cmd_pos_pan;
	ros::Subscriber p_sub_cmd_pos_tilt;
	ros::Subscriber p_sub_cmd_pos_pan32;
	ros::Subscriber p_sub_cmd_pos_tilt32;
	ros::Subscriber p_sub_cmd_pos_stamped;
	bool p_use_posestamped;
	std::string p_tf_frame_pantilt;
	tf::TransformListener* tfListener;

	JausAddress p_remote_addr;
	bool p_has_access;
	bool p_by_query;
	void pQueryCallback(const ros::TimerEvent& event);
	void pUpdateCmdPosition(double pan, double tilt);

	void pJoinStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
	void pPanFloatCallback(const std_msgs::Float64::ConstPtr& msg);
	void pTiltFloatCallback(const std_msgs::Float64::ConstPtr& msg);
	void pPanFloat32Callback(const std_msgs::Float32::ConstPtr& msg);
	void pTiltFloat32Callback(const std_msgs::Float32::ConstPtr& msg);
	void pPanTiltPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

};

};

#endif // PANTILTJOINTPOSITIONDRIVERCLIENT_RECEIVEFSM_H
