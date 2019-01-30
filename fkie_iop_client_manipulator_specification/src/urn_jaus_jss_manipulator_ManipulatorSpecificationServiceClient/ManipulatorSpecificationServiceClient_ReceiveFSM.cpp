

#include "urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient/ManipulatorSpecificationServiceClient_ReceiveFSM.h"
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_config.h>
#include <algorithm>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient
{



ManipulatorSpecificationServiceClient_ReceiveFSM::ManipulatorSpecificationServiceClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new ManipulatorSpecificationServiceClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	p_has_access = false;
	p_hz = 10.0;
	p_query_spec.getBody()->getQueryManipulatorSpecificationsRec()->setPresenceVector(255);
}



ManipulatorSpecificationServiceClient_ReceiveFSM::~ManipulatorSpecificationServiceClient_ReceiveFSM()
{
	delete context;
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_ManipulatorSpecificationServiceClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_ManipulatorSpecificationServiceClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "ManipulatorSpecificationServiceClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "ManipulatorSpecificationServiceClient_ReceiveFSM");
	iop::Config cfg("~ManipulatorSpecificationServiceClient");
	cfg.param("hz", p_hz, p_hz, false, false);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:ManipulatorSpecificationService", 2, 0);
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::handleReportManipulatorSpecificationsAction(ReportManipulatorSpecifications msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	p_query_timer.stop();
	pNotifyListeners(transportData.getAddress(), msg);
/*
	ReportManipulatorSpecifications::Body::ReportManipulatorSpecification::JointSpecificationList *jointlist = msg.getBody()->getReportManipulatorSpecification()->getJointSpecificationList();
	p_joint_names.clear();
	p_joint_efforts.clear();
	p_joint_names.push_back("first_joint");
	p_joint_efforts["first_joint"] = 0.;
	for (unsigned int index = 0; index < jointlist->getNumberOfElements(); index++) {
		std::stringstream sstm;
		sstm << "joint_" << index+2;
		p_joint_names.push_back(sstm.str());
		p_joint_efforts[sstm.str()] = 0.;
	//    ROS_INFO("  add %s joint", sstm.str().c_str());
	}
	// publish the current state of the manipulator
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
	for (unsigned int index = 0; index < p_joint_names.size(); index++) {
		msg.name.push_back(p_joint_names[index]);
		msg.velocity.push_back(0.);
	}
	p_pub_jointstates.publish(msg);
	*/
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::add_listener(iop::SpecificationListenerInterface *listener)
{
	if (listener != NULL) {
		std::vector<iop::SpecificationListenerInterface*>::iterator res = std::find(p_spec_listener.begin(), p_spec_listener.end(), listener);
		if (res == p_spec_listener.end()) {
			p_spec_listener.push_back(listener);
		}
	}
}

std::vector<std::string> ManipulatorSpecificationServiceClient_ReceiveFSM::get_joint_names(ReportManipulatorSpecifications spec)
{
	std::vector<std::string> result;
	ReportManipulatorSpecifications::Body::ReportManipulatorSpecification::JointSpecificationList *jointlist = spec.getBody()->getReportManipulatorSpecification()->getJointSpecificationList();
	std::string first_name("first_joint");
	if (spec.getBody()->getReportManipulatorSpecification()->getJointNamesList()->getNumberOfElements() > 0) {
		std::string first_spec_name = spec.getBody()->getReportManipulatorSpecification()->getJointNamesList()->getElement(0)->getDescription();
		if (!first_spec_name.empty()) {
			first_name = first_spec_name;
		}
	}
	result.push_back(first_name);
	for (unsigned int index = 0; index < jointlist->getNumberOfElements(); index++) {
		std::string joint_name;
		if (spec.getBody()->getReportManipulatorSpecification()->getJointNamesList()->getNumberOfElements() > index + 1) {
			joint_name = spec.getBody()->getReportManipulatorSpecification()->getJointNamesList()->getElement(index + 1)->getDescription();
		}
		if (joint_name.empty()) {
			std::stringstream sstm;
			sstm << "joint_" << index+2;
			joint_name = sstm.str();
		}
		result.push_back(joint_name);
	}
	return result;
}

std::vector<bool> ManipulatorSpecificationServiceClient_ReceiveFSM::get_joint_types(ReportManipulatorSpecifications spec)
{
	std::vector<bool> result;
	result.push_back(spec.getBody()->getReportManipulatorSpecification()->getFirstJointParameters()->getFieldValue());
	ReportManipulatorSpecifications::Body::ReportManipulatorSpecification::JointSpecificationList *jointlist = spec.getBody()->getReportManipulatorSpecification()->getJointSpecificationList();
	for (unsigned int index = 0; index < jointlist->getNumberOfElements(); index++) {
		result.push_back(jointlist->getElement(index)->getFieldValue());
	}
	return result;
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:ManipulatorSpecificationService") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("ManipulatorSpecificationServiceClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	sendJausMessage(p_query_spec, component);
	p_query_timer = p_nh.createTimer(ros::Duration(5), &ManipulatorSpecificationServiceClient_ReceiveFSM::pQueryCallback, this);
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	// no events for specification
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_spec, p_remote_addr);
	}
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::pNotifyListeners(JausAddress reporter, ReportManipulatorSpecifications spec)
{
	for (unsigned int i = 0; i < p_spec_listener.size(); i++) {
		p_spec_listener[i]->specification_received(reporter, spec);
	}
}

void ManipulatorSpecificationServiceClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	// no events for specification
}


};
