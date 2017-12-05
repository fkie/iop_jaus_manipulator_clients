

#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/PanTiltSpecificationServiceClient_ReceiveFSM.h"
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient
{



PanTiltSpecificationServiceClient_ReceiveFSM::PanTiltSpecificationServiceClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PanTiltSpecificationServiceClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	p_joint1_name = "pt_joint1";
	p_joint2_name = "pt_joint2";
	p_tf_frame_robot = "base_link";
	p_tf_frame_pantilt = "pantilt";
	p_remote_addr = JausAddress(0);
	p_has_access = false;
	p_query_spec.getBody()->getQueryPanTiltSpecificationsRec()->setPresenceVector(255);
}



PanTiltSpecificationServiceClient_ReceiveFSM::~PanTiltSpecificationServiceClient_ReceiveFSM()
{
	delete context;
}

void PanTiltSpecificationServiceClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PanTiltSpecificationServiceClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PanTiltSpecificationServiceClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "PanTiltSpecificationServiceClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "PanTiltSpecificationServiceClient_ReceiveFSM");
	iop::Config cfg("~PanTiltSpecificationServiceClient");
	cfg.param("joint1_name", p_joint1_name, p_joint1_name, true, false);
	cfg.param("joint2_name", p_joint2_name, p_joint2_name, true, false);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot, true, true);
	cfg.param("tf_frame_pantilt", p_tf_frame_pantilt, p_tf_frame_pantilt, true, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:PanTiltSpecificationService", 2, 0);

}

void PanTiltSpecificationServiceClient_ReceiveFSM::handleReportPanTiltSpecificationsAction(ReportPanTiltSpecifications msg, Receive::Body::ReceiveRec transportData)
{
	p_query_timer.stop();
	JausAddress sender = transportData.getAddress();
	ReportPanTiltSpecifications::Body::ReportPanTiltSpecificationsRec* rec = msg.getBody()->getReportPanTiltSpecificationsRec();
	p_static_transformStamped.header.stamp = ros::Time::now();
	p_static_transformStamped.header.frame_id = p_tf_frame_robot;
	p_static_transformStamped.child_frame_id = p_tf_frame_pantilt;
	p_static_transformStamped.transform.translation.x = 0.0;
	p_static_transformStamped.transform.translation.y = 0.0;
	p_static_transformStamped.transform.translation.z = 0.0;
	p_static_transformStamped.transform.rotation.x = 0.0;
	p_static_transformStamped.transform.rotation.y = 0.0;
	p_static_transformStamped.transform.rotation.z = 0.0;
	p_static_transformStamped.transform.rotation.w = 1.0;
	if (rec->isPanTiltCoordinateSysXValid()) {
		p_static_transformStamped.transform.translation.x = rec->getPanTiltCoordinateSysX();
	}
	if (rec->isPanTiltCoordinateSysYValid()) {
		p_static_transformStamped.transform.translation.y = rec->getPanTiltCoordinateSysY();
	}
	if (rec->isPanTiltCoordinateSysZValid()) {
		p_static_transformStamped.transform.translation.z = rec->getPanTiltCoordinateSysZ();
	}
	if (rec->isAComponentOfUnitQuaternionQValid()) {
		p_static_transformStamped.transform.rotation.x = rec->getAComponentOfUnitQuaternionQ();
	}
	if (rec->isBComponentOfUnitQuaternionQValid()) {
		p_static_transformStamped.transform.rotation.y = rec->getBComponentOfUnitQuaternionQ();
	}
	if (rec->isCComponentOfUnitQuaternionQValid()) {
		p_static_transformStamped.transform.rotation.z = rec->getCComponentOfUnitQuaternionQ();
	}
	if (rec->isDComponentOfUnitQuaternionQValid()) {
		p_static_transformStamped.transform.rotation.w = rec->getDComponentOfUnitQuaternionQ();
	}
	p_static_broadcaster.sendTransform(p_static_transformStamped);
	ROS_DEBUG_NAMED("PanTiltSpecificationServiceClient", "received pantilt specification from %s, publish static transform %s -> %s", sender.str().c_str(), p_tf_frame_robot.c_str(), p_tf_frame_pantilt.c_str());
	pNotifyListeners(sender, msg);
}

void PanTiltSpecificationServiceClient_ReceiveFSM::add_listener(iop::PanTiltSpecificationListenerInterface *listener)
{
	if (listener != NULL) {
		std::vector<iop::PanTiltSpecificationListenerInterface*>::iterator res = std::find(p_spec_listener.begin(), p_spec_listener.end(), listener);
		if (res == p_spec_listener.end()) {
			p_spec_listener.push_back(listener);
		}
	}
}

std::pair<std::string, std::string> PanTiltSpecificationServiceClient_ReceiveFSM::getJointNames()
{
	return std::make_pair(p_joint1_name, p_joint2_name);
}

void PanTiltSpecificationServiceClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:PanTiltSpecificationService") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("PanTiltSpecificationServiceClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PanTiltSpecificationServiceClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PanTiltSpecificationServiceClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void PanTiltSpecificationServiceClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	// try to get the specification from robot
	if (p_remote_addr.get() != 0) {
		p_query_timer.stop();
		ROS_INFO_NAMED("PanTiltSpecificationServiceClient", "create QUERY timer to get pantilt specification from %s", p_remote_addr.str().c_str());
		p_query_timer = p_nh.createTimer(ros::Duration(3.0), &PanTiltSpecificationServiceClient_ReceiveFSM::pQueryCallback, this);
	}
}

void PanTiltSpecificationServiceClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
}

void PanTiltSpecificationServiceClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_spec, p_remote_addr);
	}
}

void PanTiltSpecificationServiceClient_ReceiveFSM::pNotifyListeners(JausAddress reporter, ReportPanTiltSpecifications spec)
{
	for (unsigned int i = 0; i < p_spec_listener.size(); i++) {
		p_spec_listener[i]->pantilt_specification_received(reporter, spec);
	}
}

};
