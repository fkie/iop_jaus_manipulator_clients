

#include "urn_jaus_jss_manipulator_PrimitivePanTiltClient/PrimitivePanTiltClient_ReceiveFSM.h"
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_component.h>
#include <iop_component_fkie/iop_config.h>
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/PanTiltSpecificationServiceClientService.h"



using namespace JTS;
using namespace iop::ocu;
using namespace urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient;

namespace urn_jaus_jss_manipulator_PrimitivePanTiltClient
{
double PrimitivePanTiltClient_ReceiveFSM::DEFAUL_MAX_SPEED = 6;


PrimitivePanTiltClient_ReceiveFSM::PrimitivePanTiltClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PrimitivePanTiltClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	p_remote_addr = JausAddress(0);
	p_has_access = false;
	p_by_query = false;
	pPtSpecService = NULL;
	p_joint1_name = "";
	p_joint2_name = "";
	p_cmd_joint1_vel = 0.0;
	p_cmd_joint2_vel = 0.0;
	p_joint1_max_vel = DEFAUL_MAX_SPEED;
	p_joint2_max_vel = DEFAUL_MAX_SPEED;
	p_hz = 0.0;
}



PrimitivePanTiltClient_ReceiveFSM::~PrimitivePanTiltClient_ReceiveFSM()
{
	delete context;
}

void PrimitivePanTiltClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PrimitivePanTiltClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PrimitivePanTiltClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "PrimitivePanTiltClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "PrimitivePanTiltClient_ReceiveFSM");
	iop::Component &cmp = iop::Component::get_instance();
	pPtSpecService = static_cast<PanTiltSpecificationServiceClientService*>(cmp.get_service("PanTiltSpecificationServiceClient"));
	if (pPtSpecService != NULL) {
		std::pair<std::string, std::string> joint_names = pPtSpecService->pPanTiltSpecificationServiceClient_ReceiveFSM->getJointNames();
		p_joint1_name = joint_names.first;
		p_joint2_name = joint_names.second;
		pPtSpecService->pPanTiltSpecificationServiceClient_ReceiveFSM->add_listener(this);
	} else {
		throw std::runtime_error("[PanTiltJointPositionDriverClient] no PanTiltSpecificationServiceClient in configuration found! Please include its plugin first (in the list)!");
	}

	iop::Config cfg("~PrimitivePanTiltClient");
	p_sub_cmd_vel_joints = cfg.subscribe<sensor_msgs::JointState>("cmd_vel_joints", 5, &PrimitivePanTiltClient_ReceiveFSM::pJoinStateCallback, this);
	p_sub_cmd_vel_pan = cfg.subscribe<std_msgs::Float64>("cmd_vel_pan", 5, &PrimitivePanTiltClient_ReceiveFSM::pPanFloatCallback, this);
	p_sub_cmd_vel_tilt = cfg.subscribe<std_msgs::Float64>("cmd_vel_tilt", 5, &PrimitivePanTiltClient_ReceiveFSM::pTiltFloatCallback, this);
	p_sub_cmd_vel_pan32 = cfg.subscribe<std_msgs::Float32>("cmd_vel_pan32", 5, &PrimitivePanTiltClient_ReceiveFSM::pPanFloat32Callback, this);
	p_sub_cmd_vel_tilt32 = cfg.subscribe<std_msgs::Float32>("cmd_vel_tilt32", 5, &PrimitivePanTiltClient_ReceiveFSM::pTiltFloat32Callback, this);
	p_sub_cmd_twist = cfg.subscribe<geometry_msgs::TwistStamped>("cmd_vel_twist", 5, &PrimitivePanTiltClient_ReceiveFSM::pTwistCallback, this);

	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_vel_joints = cfg.advertise<sensor_msgs::JointState>("cmded_vel_joints", 5, false);
	p_pub_vel_pan = cfg.advertise<std_msgs::Float64>("cmded_vel_pan", 5, false);
	p_pub_vel_tilt = cfg.advertise<std_msgs::Float64>("cmded_vel_tilt", 5, false);
	p_pub_vel_pan32 = cfg.advertise<std_msgs::Float32>("cmded_vel_pan32", 5, false);
	p_pub_vel_tilt32 = cfg.advertise<std_msgs::Float32>("cmded_vel_tilt32", 5, false);
	p_pub_vel_twist = cfg.advertise<geometry_msgs::TwistStamped>("cmded_vel_twist", 5, false);

	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:PrimitivePanTilt", 2, 0);
}

void PrimitivePanTiltClient_ReceiveFSM::handleReportPanTiltJointEffortAction(ReportPanTiltJointEffort msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	double joint1_effort = msg.getBody()->getPanTiltJointEffortRec()->getJoint1Effort();
	double joint2_effort = msg.getBody()->getPanTiltJointEffortRec()->getJoint2Effort();
	ROS_DEBUG_NAMED("PrimitivePanTiltClient", "handle pantilt effort (%.2f, %.2f) from %s", joint1_effort, joint2_effort, sender.str().c_str());
	double joint1_vel = joint1_effort / 100.0 * p_joint1_max_vel;
	double joint2_vel = joint2_effort / 100.0 * p_joint2_max_vel;
	sensor_msgs::JointState joint_state;
	if (!p_joint1_name.empty()) {
		joint_state.name.push_back(p_joint1_name);
		joint_state.position.push_back(joint1_vel);
	}
	if (!p_joint2_name.empty()) {
		joint_state.name.push_back(p_joint2_name);
		joint_state.position.push_back(joint2_vel);
	}
	p_pub_vel_joints.publish(joint_state);
	// send to Float64 topics
	std_msgs::Float64 vel_pan;
	vel_pan.data = joint1_vel;
	p_pub_vel_pan.publish(vel_pan);
	std_msgs::Float64 vel_tilt;
	vel_tilt.data = joint2_vel;
	p_pub_vel_tilt.publish(vel_tilt);
	// send cmd to Float32 topics
	std_msgs::Float32 vel_pan32;
	vel_pan32.data = joint1_vel;
	p_pub_vel_pan32.publish(vel_pan32);
	std_msgs::Float32 vel_tilt32;
	vel_tilt32.data = joint2_vel;
	p_pub_vel_tilt32.publish(vel_tilt32);
	geometry_msgs::TwistStamped ros_twist;
	ros_twist.header.stamp = ros::Time::now();
	ros_twist.twist.angular.y = joint2_vel;
	ros_twist.twist.angular.z = joint1_vel;
	p_pub_vel_twist.publish(ros_twist);
}

void PrimitivePanTiltClient_ReceiveFSM::pantilt_specification_received(JausAddress reporter, urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::ReportPanTiltSpecifications spec)
{
	if (p_remote_addr == reporter && p_remote_addr.get() != 0) {
		p_joint1_max_vel = spec.getBody()->getReportPanTiltSpecificationsRec()->getJoint1MaxSpeed();
		p_joint2_max_vel = spec.getBody()->getReportPanTiltSpecificationsRec()->getJoint2MaxSpeed();
		ROS_DEBUG_NAMED("PrimitivePanTiltClient", "specification received; max speed joint1: %.2f, joint2: %.2f", p_joint1_max_vel, p_joint2_max_vel);
		if (p_by_query) {
			p_query_timer.stop();
			if (p_hz > 0) {
				ROS_INFO_NAMED("PrimitivePanTiltClient", "create QUERY timer to get pantilt effort from %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &PrimitivePanTiltClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("PrimitivePanTiltClient", "invalid hz %.2f for QUERY timer to get pantilt effort from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("PrimitivePanTiltClient", "create EVENT to get pantilt effort from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_effort, p_hz);
		}
	}
}

void PrimitivePanTiltClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:PrimitivePanTilt") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("PrimitivePanTiltClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PrimitivePanTiltClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PrimitivePanTiltClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
	p_joint1_max_vel = DEFAUL_MAX_SPEED;
	p_joint2_max_vel = DEFAUL_MAX_SPEED;
	ROS_DEBUG_NAMED("PrimitivePanTiltClient", "default max speed joint1: %.2f, joint2: %.2f", p_joint1_max_vel, p_joint2_max_vel);
	p_cmd_joint1_vel = 0.0;
	p_cmd_joint2_vel = 0.0;
}

void PrimitivePanTiltClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	// query is created after specification is received
}

void PrimitivePanTiltClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("PrimitivePanTiltClient", "cancel EVENT for pantilt effort on %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_effort);
	}
}

void PrimitivePanTiltClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_effort, p_remote_addr);
	}
}

void PrimitivePanTiltClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportPanTiltJointEffort report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportPanTiltJointEffortAction(report, transport_data);
}

double PrimitivePanTiltClient_ReceiveFSM::pNormalize(double value, double value_max)
{
	double result = 0;
	if (std::abs(value_max) > 0) {
		result = value / value_max * 100.0;
		if (result > 100) {
			result = 100;
		} else if (result < -100.0) {
			result = -100.0;
		}
	}
	return result;
}

void PrimitivePanTiltClient_ReceiveFSM::pUpdateCmdVelocity(double pan, double tilt)
{
	lock_type lock(p_mutex);
	if (p_cmd_joint1_vel != pan) {
		p_cmd_joint1_vel = pan;
	}
	if (p_cmd_joint2_vel != tilt) {
		p_cmd_joint2_vel = tilt;
	}
	if (p_remote_addr.get() != 0) {
		// send command to the controlled service
		SetPanTiltJointEffort cmd_msg;
		double val1 = pNormalize(pan, p_joint1_max_vel);
		double val2 = pNormalize(tilt, p_joint2_max_vel);
		cmd_msg.getBody()->getPanTiltJointEffortRec()->setJoint1Effort(val1);
		cmd_msg.getBody()->getPanTiltJointEffortRec()->setJoint2Effort(val2);
		sendJausMessage(cmd_msg, p_remote_addr);
	}
}

void PrimitivePanTiltClient_ReceiveFSM::pJoinStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	if (p_remote_addr.get() != 0) {
		double pan = p_cmd_joint1_vel;
		double tilt = p_cmd_joint2_vel;
		for (unsigned int index = 0; index < joint_state->name.size(); index++) {
			if (joint_state->name[index].compare(p_joint1_name) == 0) {
				pan = joint_state->position[index];
			} else if (joint_state->name[index].compare(p_joint2_name) == 0) {
				tilt = joint_state->position[index];
			}
		}
		pUpdateCmdVelocity(pan, tilt);
	}
}

void PrimitivePanTiltClient_ReceiveFSM::pPanFloatCallback(const std_msgs::Float64::ConstPtr& msg)
{
	pUpdateCmdVelocity(msg->data, p_cmd_joint2_vel);
}

void PrimitivePanTiltClient_ReceiveFSM::pTiltFloatCallback(const std_msgs::Float64::ConstPtr& msg)
{
	pUpdateCmdVelocity(p_cmd_joint1_vel, msg->data);
}

void PrimitivePanTiltClient_ReceiveFSM::pPanFloat32Callback(const std_msgs::Float32::ConstPtr& msg)
{
	pUpdateCmdVelocity(msg->data, p_cmd_joint2_vel);
}

void PrimitivePanTiltClient_ReceiveFSM::pTiltFloat32Callback(const std_msgs::Float32::ConstPtr& msg)
{
	pUpdateCmdVelocity(p_cmd_joint1_vel, msg->data);
}

void PrimitivePanTiltClient_ReceiveFSM::pTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	pUpdateCmdVelocity(msg->twist.angular.z, msg->twist.angular.y);
}

};
