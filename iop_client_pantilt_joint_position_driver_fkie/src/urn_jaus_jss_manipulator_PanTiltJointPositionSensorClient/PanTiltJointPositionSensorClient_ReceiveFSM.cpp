

#include "urn_jaus_jss_manipulator_PanTiltJointPositionSensorClient/PanTiltJointPositionSensorClient_ReceiveFSM.h"
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_component.h>
#include <iop_component_fkie/iop_config.h>
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/PanTiltSpecificationServiceClientService.h"


using namespace JTS;
using namespace iop::ocu;
using namespace urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient;

namespace urn_jaus_jss_manipulator_PanTiltJointPositionSensorClient
{



PanTiltJointPositionSensorClient_ReceiveFSM::PanTiltJointPositionSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PanTiltJointPositionSensorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	p_remote_addr = JausAddress(0);
	p_has_access = false;
	p_by_query = false;
	pPtSpecService = NULL;
	p_joint1_name = "";
	p_joint2_name = "";
	p_hz = 10.0;
	p_use_posestamped = false;
	p_tf_frame_pantilt = "";
}



PanTiltJointPositionSensorClient_ReceiveFSM::~PanTiltJointPositionSensorClient_ReceiveFSM()
{
	delete context;
}

void PanTiltJointPositionSensorClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PanTiltJointPositionSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PanTiltJointPositionSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "PanTiltJointPositionSensorClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "PanTiltJointPositionSensorClient_ReceiveFSM");
	iop::Component &cmp = iop::Component::get_instance();
	pPtSpecService = static_cast<PanTiltSpecificationServiceClientService*>(cmp.get_service("PanTiltSpecificationServiceClient"));
	if (pPtSpecService != NULL) {
		std::pair<std::string, std::string> joint_names = pPtSpecService->pPanTiltSpecificationServiceClient_ReceiveFSM->getJointNames();
		p_joint1_name = joint_names.first;
		p_joint2_name = joint_names.second;
	} else {
		throw std::runtime_error("[PanTiltJointPositionSensorClient] no PanTiltSpecificationServiceClient in configuration found! Please include its plugin first (in the list)!");
	}

	iop::Config cfg("~PanTiltJointPositionSensorClient");
	cfg.param("hz", p_hz, p_hz, false, false);
	cfg.param("use_posestamped", p_use_posestamped, p_use_posestamped);
	if (p_use_posestamped) {
		cfg.param("tf_frame_pantilt", p_tf_frame_pantilt, p_tf_frame_pantilt);
		p_sub_pos_stamped = cfg.advertise<geometry_msgs::PoseStamped>("pos_pantilt", 5, false);
	}
	p_pub_pos_joints = cfg.advertise<sensor_msgs::JointState>("pos_joints", 5, false);
	p_pub_pos_pan = cfg.advertise<std_msgs::Float64>("pos_pan", 5, false);
	p_pub_pos_tilt = cfg.advertise<std_msgs::Float64>("pos_tilt", 5, false);
	p_pub_pos_pan32 = cfg.advertise<std_msgs::Float32>("pos_pan32", 5, false);
	p_pub_pos_tilt32 = cfg.advertise<std_msgs::Float32>("pos_tilt32", 5, false);

	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:PanTiltJointPositionSensor", 2, 0);
}

void PanTiltJointPositionSensorClient_ReceiveFSM::handleReportPanTiltJointPositionsAction(ReportPanTiltJointPositions msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	double joint1_position = msg.getBody()->getPanTiltJointPositionRec()->getJoint1Position();
	double joint2_position = msg.getBody()->getPanTiltJointPositionRec()->getJoint2Position();
	ROS_DEBUG_NAMED("PanTiltJointPositionSensorClient", "handle pantilt position (%.2f, %.2f) from %s", joint1_position, joint2_position, sender.str().c_str());
	sensor_msgs::JointState joint_state;
	if (!p_joint1_name.empty()) {
		joint_state.name.push_back(p_joint1_name);
		joint_state.position.push_back(joint1_position);
	}
	if (!p_joint2_name.empty()) {
		joint_state.name.push_back(p_joint2_name);
		joint_state.position.push_back(joint2_position);
	}
	p_pub_pos_joints.publish(joint_state);
	// send to Float64 topics
	std_msgs::Float64 pos_pan;
	pos_pan.data = joint1_position;
	p_pub_pos_pan.publish(pos_pan);
	std_msgs::Float64 pos_tilt;
	pos_tilt.data = joint2_position;
	p_pub_pos_tilt.publish(pos_tilt);
	// send cmd to Float32 topics
	std_msgs::Float32 pos_pan32;
	pos_pan32.data = joint1_position;
	p_pub_pos_pan32.publish(pos_pan32);
	std_msgs::Float32 pos_tilt32;
	pos_tilt32.data = joint2_position;
	p_pub_pos_tilt32.publish(pos_tilt32);
	if (p_use_posestamped) {
		geometry_msgs::PoseStamped cmd_pose;
		tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, joint2_position, joint1_position);
		cmd_pose.header.frame_id = p_tf_frame_pantilt;
		cmd_pose.header.stamp = ros::Time::now();
		cmd_pose.pose.position.x = 0;
		cmd_pose.pose.position.y = 0;
		cmd_pose.pose.position.z = 0;
		cmd_pose.pose.orientation.x = quat.x();
		cmd_pose.pose.orientation.y = quat.y();
		cmd_pose.pose.orientation.z = quat.z();
		cmd_pose.pose.orientation.w = quat.w();

		p_sub_pos_stamped.publish(cmd_pose);
	}
}

void PanTiltJointPositionSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:PanTiltJointPositionSensor") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("PanTiltJointPositionSensorClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PanTiltJointPositionSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PanTiltJointPositionSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void PanTiltJointPositionSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	ROS_INFO_NAMED("PanTiltJointPositionSensorClient", "create EVENT to get pantilt joint positions from %s", p_remote_addr.str().c_str());
	pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_pos, p_hz);
}

void PanTiltJointPositionSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("PanTiltJointPositionSensorClient", "cancel EVENT for pantilt joint positions on %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_pos);
	}
}

void PanTiltJointPositionSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_pos, p_remote_addr);
	}
}

void PanTiltJointPositionSensorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportPanTiltJointPositions report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportPanTiltJointPositionsAction(report, transport_data);
}

};
