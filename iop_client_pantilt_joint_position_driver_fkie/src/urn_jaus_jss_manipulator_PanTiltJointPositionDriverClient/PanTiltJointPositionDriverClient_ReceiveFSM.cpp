

#include "urn_jaus_jss_manipulator_PanTiltJointPositionDriverClient/PanTiltJointPositionDriverClient_ReceiveFSM.h"
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_component.h>
#include <iop_component_fkie/iop_config.h>
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/PanTiltSpecificationServiceClientService.h"



using namespace JTS;
using namespace iop::ocu;
using namespace urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient;

namespace urn_jaus_jss_manipulator_PanTiltJointPositionDriverClient
{



PanTiltJointPositionDriverClient_ReceiveFSM::PanTiltJointPositionDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PanTiltJointPositionDriverClient_ReceiveFSMContext(*this);

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
	p_cmd_joint1_position = 0.0;
	p_cmd_joint2_position = 0.0;
}



PanTiltJointPositionDriverClient_ReceiveFSM::~PanTiltJointPositionDriverClient_ReceiveFSM()
{
	delete context;
}

void PanTiltJointPositionDriverClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PanTiltJointPositionDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PanTiltJointPositionDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "PanTiltJointPositionDriverClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "PanTiltJointPositionDriverClient_ReceiveFSM");
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

	iop::Config cfg("~PanTiltJointPositionDriverClient");
	p_sub_cmd_pos_joints = cfg.subscribe<sensor_msgs::JointState>("cmd_pos_joints", 5, &PanTiltJointPositionDriverClient_ReceiveFSM::pJoinStateCallback, this);
	p_sub_cmd_pos_pan = cfg.subscribe<std_msgs::Float64>("cmd_pos_pan", 5, &PanTiltJointPositionDriverClient_ReceiveFSM::pPanFloatCallback, this);
	p_sub_cmd_pos_tilt = cfg.subscribe<std_msgs::Float64>("cmd_pos_tilt", 5, &PanTiltJointPositionDriverClient_ReceiveFSM::pTiltFloatCallback, this);
	p_sub_cmd_pos_pan32 = cfg.subscribe<std_msgs::Float32>("cmd_pos_pan32", 5, &PanTiltJointPositionDriverClient_ReceiveFSM::pPanFloat32Callback, this);
	p_sub_cmd_pos_tilt32 = cfg.subscribe<std_msgs::Float32>("cmd_pos_tilt32", 5, &PanTiltJointPositionDriverClient_ReceiveFSM::pTiltFloat32Callback, this);

	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:PanTiltJointPositionDriver", 2, 0);

}

void PanTiltJointPositionDriverClient_ReceiveFSM::handleReportCommandedPanTiltJointPositionsAction(ReportCommandedPanTiltJointPositions msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void PanTiltJointPositionDriverClient_ReceiveFSM::pantilt_specification_received(JausAddress reporter, urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::ReportPanTiltSpecifications spec)
{
	ROS_WARN_STREAM_NAMED("PanTiltJointPositionDriverClient", "specification received, but not handled in PanTiltJointPositionDriverClient -> not implemented");
}

void PanTiltJointPositionDriverClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:PanTiltJointPositionDriver") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("PanTiltJointPositionDriverClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PanTiltJointPositionDriverClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PanTiltJointPositionDriverClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void PanTiltJointPositionDriverClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	// we request no commanded positions. Current position are requested in pantilt position sensor
}

void PanTiltJointPositionDriverClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
}

void PanTiltJointPositionDriverClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		// we request no commanded positions. Current position are requested in pantilt position sensor
	}
}

void PanTiltJointPositionDriverClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportCommandedPanTiltJointPositions report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportCommandedPanTiltJointPositionsAction(report, transport_data);
}

void PanTiltJointPositionDriverClient_ReceiveFSM::pUpdateCmdPosition(double pan, double tilt)
{
	lock_type lock(p_mutex);
	if (p_cmd_joint1_position != pan) {
		p_cmd_joint1_position = pan;
	}
	if (p_cmd_joint2_position != tilt) {
		p_cmd_joint2_position = tilt;
	}
	if (p_remote_addr.get() != 0) {
		// send command to the controlled service
		SetPanTiltJointPositions cmd_msg;
		cmd_msg.getBody()->getPanTiltJointPositionRec()->setJoint1Position(p_cmd_joint1_position);
		cmd_msg.getBody()->getPanTiltJointPositionRec()->setJoint2Position(p_cmd_joint2_position);
		sendJausMessage(cmd_msg, p_remote_addr);
	}
}

void PanTiltJointPositionDriverClient_ReceiveFSM::pJoinStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	if (p_remote_addr.get() != 0) {
		double pan = p_cmd_joint1_position;
		double tilt = p_cmd_joint2_position;
		for (unsigned int index = 0; index < joint_state->name.size(); index++) {
			if (joint_state->name[index].compare(p_joint1_name) == 0) {
				pan = joint_state->position[index];
			} else if (joint_state->name[index].compare(p_joint2_name) == 0) {
				tilt = joint_state->position[index];
			}
		}
		pUpdateCmdPosition(pan, tilt);
	}
}

void PanTiltJointPositionDriverClient_ReceiveFSM::pPanFloatCallback(const std_msgs::Float64::ConstPtr& msg)
{
	pUpdateCmdPosition(msg->data, p_cmd_joint2_position);
}

void PanTiltJointPositionDriverClient_ReceiveFSM::pTiltFloatCallback(const std_msgs::Float64::ConstPtr& msg)
{
	pUpdateCmdPosition(p_cmd_joint1_position, msg->data);
}

void PanTiltJointPositionDriverClient_ReceiveFSM::pPanFloat32Callback(const std_msgs::Float32::ConstPtr& msg)
{
	pUpdateCmdPosition(msg->data, p_cmd_joint2_position);
}

void PanTiltJointPositionDriverClient_ReceiveFSM::pTiltFloat32Callback(const std_msgs::Float32::ConstPtr& msg)
{
	pUpdateCmdPosition(p_cmd_joint1_position, msg->data);
}

};
