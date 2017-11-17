

#include "urn_jaus_jss_manipulator_PrimitiveEndEffectorClient/PrimitiveEndEffectorClient_ReceiveFSM.h"
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_manipulator_PrimitiveEndEffectorClient
{



PrimitiveEndEffectorClient_ReceiveFSM::PrimitiveEndEffectorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PrimitiveEndEffectorClient_ReceiveFSMContext(*this);

	p_has_access = false;
	p_query_state = 0;
	p_by_query = false;
	p_hz = 10.0;

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
}



PrimitiveEndEffectorClient_ReceiveFSM::~PrimitiveEndEffectorClient_ReceiveFSM()
{
	delete context;
}

void PrimitiveEndEffectorClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PrimitiveEndEffectorClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PrimitiveEndEffectorClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "PrimitiveEndEffectorClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "PrimitiveEndEffectorClient_ReceiveFSM");
	iop::Config cfg("~RangeSensorClient");
	cfg.param("hz", p_hz, p_hz, false, false);
	cfg.param("gripper_joint", p_gripper_joint, std::string("gripperWidth_joint"));
	// subscribe to ROS joint state commands
	p_sub_jointstates = p_nh.subscribe<sensor_msgs::JointState>("cmd_joint_velocities", 1, &PrimitiveEndEffectorClient_ReceiveFSM::pRosCmdJointState, this);
	p_sub_cmd_vel = p_nh.subscribe<std_msgs::Float64>("gripper_velocity_controller/command", 1, &PrimitiveEndEffectorClient_ReceiveFSM::pRosCmdVelocity, this);
	p_pub_jointstates = p_nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
	p_pub_float = p_nh.advertise<std_msgs::Float64>("effort", 1, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:PrimitiveEndEffector", 2, 0);
}

void PrimitiveEndEffectorClient_ReceiveFSM::handleReportEndEffectorEffortAction(ReportEndEffectorEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	sensor_msgs::JointState ros_msg;
	ros_msg.header.stamp = ros::Time::now();
	ros_msg.name.push_back(p_gripper_joint);
	double effort = msg.getBody()->getEndEffectorEffortRec()->getEndEffectorEffort();
	float val = (int)(effort * 100) / 100.;
	ros_msg.velocity.push_back(val);
	p_pub_jointstates.publish(ros_msg);
	std_msgs::Float64 ros_fmsg;
	ros_fmsg.data = val;
	p_pub_float.publish(ros_fmsg);
}

void PrimitiveEndEffectorClient_ReceiveFSM::handleReportEndEffectorSpecificationAction(ReportEndEffectorSpecification msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PrimitiveEndEffectorClient", "reportEndEffectorSpecificationAction from %s", sender.str().c_str());
	JausAddress manipulator_id(msg.getBody()->getReportEndEffectorSpecificationRec()->getParentId());
	p_manipulator_id = manipulator_id;
	// publish the current state of the endeffector
	sensor_msgs::JointState ros_msg;
	ros_msg.header.stamp = ros::Time::now();
	ros_msg.name.push_back(p_gripper_joint);
	ros_msg.velocity.push_back(0.0);
	p_pub_jointstates.publish(ros_msg);
	p_query_state = 1;
	// create event or timer for queries
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			p_query_timer.stop();
			if (p_hz > 0) {
				ROS_INFO_NAMED("PrimitiveEndEffectorClient", "create QUERY timer to get endeffector effort from %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &PrimitiveEndEffectorClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("PrimitiveEndEffectorClient", "invalid hz %.2f for QUERY timer to get endeffector effort from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("PrimitiveEndEffectorClient", "create EVENT to get endeffector effort from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_effort, p_hz);
		}
	}
}

void PrimitiveEndEffectorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:PrimitiveEndEffector") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("PrimitiveEndEffectorClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PrimitiveEndEffectorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PrimitiveEndEffectorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void PrimitiveEndEffectorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	sendJausMessage(p_query_spec, component);
	p_query_timer = p_nh.createTimer(ros::Duration(5), &PrimitiveEndEffectorClient_ReceiveFSM::pQueryCallback, this);
}

void PrimitiveEndEffectorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("PrimitiveEndEffectorClient", "cancel EVENT for primitive end effector on %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_effort);
	}
	p_query_state = 0;
}

void PrimitiveEndEffectorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		if (p_query_state == 0) {
			sendJausMessage(p_query_spec, p_remote_addr);
		} else {
			sendJausMessage(p_query_effort, p_remote_addr);
		}
	}
}

void PrimitiveEndEffectorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportEndEffectorEffort report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportEndEffectorEffortAction(report, transport_data);
}

void PrimitiveEndEffectorClient_ReceiveFSM::pRosCmdJointState(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	if (p_remote_addr.get() != 0) {
		SetEndEffectorEffort msg;
		int jname_idx = pGetNameIndexFromJointState(joint_state, p_gripper_joint);
		SetEndEffectorEffort::Body::EndEffectorEffortRec value;
		if (jname_idx > -1) {
			// joint name in message found, set the velocity value
			double vel = joint_state->velocity[jname_idx];
			// TODO: make vel relative; we need the max and min values of endeffector specification
			value.setEndEffectorEffort(vel);
		} else {
			// not in list -> set to zero
			value.setEndEffectorEffort(0.);
		}
		msg.getBody()->setEndEffectorEffortRec(value);
		sendJausMessage(msg, p_remote_addr);
	}
}

void PrimitiveEndEffectorClient_ReceiveFSM::pRosCmdVelocity(const std_msgs::Float64::ConstPtr& cmd_vel)
{
	if (p_remote_addr.get() != 0) {
		SetEndEffectorEffort msg;
		msg.getBody()->getEndEffectorEffortRec()->setEndEffectorEffort(cmd_vel->data);
		sendJausMessage(msg, p_remote_addr);
	}
}

int PrimitiveEndEffectorClient_ReceiveFSM::pGetNameIndexFromJointState(const sensor_msgs::JointState::ConstPtr& joint_state, std::string name)
{
	int result = -1;
	for (unsigned int index = 0; index < joint_state->name.size(); index++) {
		if (joint_state->name[index].compare(name) == 0) {
			result = index;
			break;
		}
	}
	return result;
}


};
