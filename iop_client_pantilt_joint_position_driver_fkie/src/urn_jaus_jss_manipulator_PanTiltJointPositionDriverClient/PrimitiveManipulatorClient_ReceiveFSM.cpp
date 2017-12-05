

#include "urn_jaus_jss_manipulator_PrimitiveManipulatorClient/PrimitiveManipulatorClient_ReceiveFSM.h"
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_component.h>
#include <iop_component_fkie/iop_config.h>



using namespace JTS;
using namespace iop::ocu;
using namespace urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient;

namespace urn_jaus_jss_manipulator_PrimitiveManipulatorClient
{



PrimitiveManipulatorClient_ReceiveFSM::PrimitiveManipulatorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PrimitiveManipulatorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	pManiSpecService = NULL;

	p_has_access = false;
	p_query_state = 0;
	p_by_query = false;
	p_hz = 10.0;
}



PrimitiveManipulatorClient_ReceiveFSM::~PrimitiveManipulatorClient_ReceiveFSM()
{
	delete context;
}

void PrimitiveManipulatorClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PrimitiveManipulatorClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PrimitiveManipulatorClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "PrimitiveManipulatorClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "PrimitiveManipulatorClient_ReceiveFSM");
	iop::Component &cmp = iop::Component::get_instance();
	pManiSpecService = static_cast<ManipulatorSpecificationServiceClientService*>(cmp.get_service("ManipulatorSpecificationServiceClient"));
	if (pManiSpecService != NULL) {
		pManiSpecService->pManipulatorSpecificationServiceClient_ReceiveFSM->add_listener(this);
	} else {
		throw std::runtime_error("[PrimitiveManipulatorClient] no ManipulatorSpecificationServiceClient in configuration found! Please include its plugin first (in the list)!");
	}
	iop::Config cfg("~PrimitiveManipulatorClient");
	cfg.param("hz", p_hz, p_hz, false, false);
	// subscribe to ROS joint state commands
	p_sub_jointstates = p_nh.subscribe<sensor_msgs::JointState>("cmd_joint_velocities", 1, &PrimitiveManipulatorClient_ReceiveFSM::pRosCmdJointState, this);
	p_sub_cmd_vel = p_nh.subscribe<std_msgs::Float64MultiArray>("velocity_controller/command", 1, &PrimitiveManipulatorClient_ReceiveFSM::pRosCmdVelocity, this);
	p_pub_jointstates = p_nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
	p_pub_float = p_nh.advertise<std_msgs::Float64MultiArray>("velocity_controller/effort", 1, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:PrimitiveManipulator", 2, 0);
}

void PrimitiveManipulatorClient_ReceiveFSM::handleReportJointEffortAction(ReportJointEffort msg, Receive::Body::ReceiveRec transportData)
{
	sensor_msgs::JointState ros_msg;
	std_msgs::Float64MultiArray ros_msg_float;
	ros_msg.header.stamp = ros::Time::now();
	for (unsigned int index = 0; index < p_joint_names.size(); index++) {
		ros_msg.name.push_back(p_joint_names[index]);
		double effort = 0.0;
		if (index < msg.getBody()->getJointEffortList()->getNumberOfElements()) {
			double re = msg.getBody()->getJointEffortList()->getElement(index)->getJointEffort();
			effort = (int)(re * 100) / 100.0;
		}
		ros_msg.velocity.push_back(effort);
		ros_msg_float.data.push_back(effort);
	}
	p_pub_jointstates.publish(ros_msg);
	p_pub_float.publish(ros_msg_float);
}

void PrimitiveManipulatorClient_ReceiveFSM::specification_received(JausAddress reporter, ReportManipulatorSpecifications spec)
{
	p_joint_names.clear();
	p_joint_names = pManiSpecService->pManipulatorSpecificationServiceClient_ReceiveFSM->get_joint_names(spec);
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			p_query_timer.stop();
			if (p_hz > 0) {
				ROS_INFO_NAMED("PrimitiveManipulatorClient", "create QUERY timer to get manipulator effort from %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &PrimitiveManipulatorClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("PrimitiveManipulatorClient", "invalid hz %.2f for QUERY timer to get manipulator effort from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("PrimitiveManipulatorClient", "create EVENT to get manipulator effort from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_effort, p_hz);
		}
	}
}

void PrimitiveManipulatorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:PrimitiveManipulator") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("PrimitiveManipulatorClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PrimitiveManipulatorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PrimitiveManipulatorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
	p_joint_names.clear();
}

void PrimitiveManipulatorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	// we wait for specification. After it will be received, specification_received() will be called end we start the events
}

void PrimitiveManipulatorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("PrimitiveManipulatorClient", "cancel EVENT for manipulator effort on %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_effort);
	}
	p_query_state = 0;
}

void PrimitiveManipulatorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_effort, p_remote_addr);
	}
}

void PrimitiveManipulatorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportJointEffort report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportJointEffortAction(report, transport_data);
}

void PrimitiveManipulatorClient_ReceiveFSM::pRosCmdJointState(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	if (p_remote_addr.get() != 0) {
		SetJointEffort msg;
		for (unsigned int index = 0; index < p_joint_names.size(); index++) {
			std::string jname = p_joint_names[index];
			int jname_idx = pGetNameIndexFromJointState(joint_state, jname);
			SetJointEffort::Body::JointEffortList::JointEffortRec value;
			if (jname_idx > -1) {
				// joint name in message found, set the velocity value
				double vel = joint_state->velocity[jname_idx];
				// TODO: make vel relative; we need the max and min values of manipulator specification
				value.setJointEffort(vel);
			} else {
				// not in list -> set to zero
				value.setJointEffort(0.);
			}
			msg.getBody()->getJointEffortList()->addElement(value);
		}
		sendJausMessage(msg, p_remote_addr);
	}
}

void PrimitiveManipulatorClient_ReceiveFSM::pRosCmdVelocity(const std_msgs::Float64MultiArray::ConstPtr& cmd_vel)
{
	if (p_remote_addr.get() != 0) {
		SetJointEffort msg;
		for (unsigned int index = 0; index < p_joint_names.size(); index++) {
			SetJointEffort::Body::JointEffortList::JointEffortRec value;
			if (index < cmd_vel->data.size()) {
				// joint name in message found, set the velocity value
				double vel = cmd_vel->data[index];
				// TODO: make vel relative; we need the max and min values of manipulator specification
				value.setJointEffort(vel);
			} else {
				// not in list -> set to zero
				value.setJointEffort(0.);
			}
			msg.getBody()->getJointEffortList()->addElement(value);
		}
		sendJausMessage(msg, p_remote_addr);
	}
}

int PrimitiveManipulatorClient_ReceiveFSM::pGetNameIndexFromJointState(const sensor_msgs::JointState::ConstPtr& joint_state, std::string name)
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
