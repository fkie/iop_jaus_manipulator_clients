

#include "urn_jaus_jss_manipulator_ManipulatorJointPositionSensorClient/ManipulatorJointPositionSensorClient_ReceiveFSM.h"
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_component.h>
#include <fkie_iop_component/iop_config.h>


using namespace JTS;
using namespace iop::ocu;
using namespace urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient;

namespace urn_jaus_jss_manipulator_ManipulatorJointPositionSensorClient
{



ManipulatorJointPositionSensorClient_ReceiveFSM::ManipulatorJointPositionSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new ManipulatorJointPositionSensorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	pManiSpecService = NULL;
	p_has_access = false;
	p_query_state = 0;
	p_by_query = false;
	p_hz = 10.0;
}



ManipulatorJointPositionSensorClient_ReceiveFSM::~ManipulatorJointPositionSensorClient_ReceiveFSM()
{
	delete context;
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_ManipulatorJointPositionSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_ManipulatorJointPositionSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "ManipulatorJointPositionSensorClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "ManipulatorJointPositionSensorClient_ReceiveFSM");
	iop::Component &cmp = iop::Component::get_instance();
	pManiSpecService = static_cast<ManipulatorSpecificationServiceClientService*>(cmp.get_service("ManipulatorSpecificationServiceClient"));
	if (pManiSpecService != NULL) {
		pManiSpecService->pManipulatorSpecificationServiceClient_ReceiveFSM->add_listener(this);
	} else {
		throw std::runtime_error("[PrimitiveManipulatorClient] no ManipulatorSpecificationServiceClient in configuration found! Please include its plugin first (in the list)!");
	}
	iop::Config cfg("~ManipulatorJointPositionSensorClient");
	cfg.param("hz", p_hz, p_hz, false, false);
	// subscribe to ROS joint state commands
	p_pub_jointstates = p_nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
	p_pub_float = p_nh.advertise<std_msgs::Float64MultiArray>("velocity_controller/effort", 1, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:ManipulatorJointPositionSensor", 2, 0);

}

void ManipulatorJointPositionSensorClient_ReceiveFSM::handleReportJointPositionsAction(ReportJointPositions msg, Receive::Body::ReceiveRec transportData)
{
	sensor_msgs::JointState ros_msg;
	std_msgs::Float64MultiArray ros_msg_float;
	ros_msg.header.stamp = ros::Time::now();
	for (unsigned int index = 0; index < p_joint_names.size(); index++) {
		ros_msg.name.push_back(p_joint_names[index]);
		double pose = 0.0;
		if (index < msg.getBody()->getJointPositionList()->getNumberOfElements()) {
			if (p_joint_types[index]) { // primatic
				pose = msg.getBody()->getJointPositionList()->getElement(index)->getJointPosition()->getRadianAsUnsignedIntegerAt0();
			} else {
				pose = msg.getBody()->getJointPositionList()->getElement(index)->getJointPosition()->getMeterAsUnsignedIntegerAt1();
			}
		}
		ros_msg.velocity.push_back(pose);
		ros_msg_float.data.push_back(pose);
	}
	p_pub_jointstates.publish(ros_msg);
	p_pub_float.publish(ros_msg_float);
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::specification_received(JausAddress reporter, ReportManipulatorSpecifications spec)
{
	p_joint_names.clear();
	p_joint_types.clear();
	p_joint_names = pManiSpecService->pManipulatorSpecificationServiceClient_ReceiveFSM->get_joint_names(spec);
	p_joint_types = pManiSpecService->pManipulatorSpecificationServiceClient_ReceiveFSM->get_joint_types(spec);
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			p_query_timer.stop();
			if (p_hz > 0) {
				ROS_INFO_NAMED("ManipulatorJointPositionSensorClient", "create QUERY timer to get manipulator effort from %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &ManipulatorJointPositionSensorClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("ManipulatorJointPositionSensorClient", "invalid hz %.2f for QUERY timer to get manipulator effort from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("ManipulatorJointPositionSensorClient", "create EVENT to get manipulator effort from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_joints, p_hz);
		}
	}
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:ManipulatorJointPositionSensor") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("ManipulatorJointPositionSensorClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
	p_joint_names.clear();
	p_joint_types.clear();
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	// we have for specification. After it will be received, specification_received() will be called end we start the events
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("ManipulatorJointPositionSensorClient", "cancel EVENT for manipulator joint position on %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_joints);
	}
	p_query_state = 0;
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_joints, p_remote_addr);
	}
}

void ManipulatorJointPositionSensorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportJointPositions report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportJointPositionsAction(report, transport_data);
}

int ManipulatorJointPositionSensorClient_ReceiveFSM::pGetNameIndexFromJointState(const sensor_msgs::JointState::ConstPtr& joint_state, std::string name)
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
