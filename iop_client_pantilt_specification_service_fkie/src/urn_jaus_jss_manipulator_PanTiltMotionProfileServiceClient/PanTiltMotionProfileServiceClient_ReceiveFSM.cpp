

#include "urn_jaus_jss_manipulator_PanTiltMotionProfileServiceClient/PanTiltMotionProfileServiceClient_ReceiveFSM.h"
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_component.h>
#include <iop_component_fkie/iop_config.h>


using namespace JTS;
using namespace iop::ocu;
using namespace urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient;

namespace urn_jaus_jss_manipulator_PanTiltMotionProfileServiceClient
{



PanTiltMotionProfileServiceClient_ReceiveFSM::PanTiltMotionProfileServiceClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PanTiltMotionProfileServiceClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_remote_addr = JausAddress(0);
	p_has_access = false;
	pPtSpecService = NULL;
}



PanTiltMotionProfileServiceClient_ReceiveFSM::~PanTiltMotionProfileServiceClient_ReceiveFSM()
{
	delete context;
}

void PanTiltMotionProfileServiceClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PanTiltMotionProfileServiceClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PanTiltMotionProfileServiceClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "PanTiltMotionProfileServiceClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "PanTiltMotionProfileServiceClient_ReceiveFSM");
	iop::Component &cmp = iop::Component::get_instance();
	pPtSpecService = static_cast<PanTiltSpecificationServiceClientService*>(cmp.get_service("PanTiltSpecificationServiceClient"));
	if (pPtSpecService != NULL) {
		pPtSpecService->pPanTiltSpecificationServiceClient_ReceiveFSM->add_listener(this);
	} else {
		throw std::runtime_error("[PanTiltMotionProfileServiceClient] no PanTiltSpecificationServiceClient in configuration found! Please include its plugin first (in the list)!");
	}
	iop::Config cfg("~PanTiltMotionProfileServiceClient");
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:manipulator:PanTiltMotionProfileService", 2, 0);

}

void PanTiltMotionProfileServiceClient_ReceiveFSM::handleReportPanTiltMotionProfileAction(ReportPanTiltMotionProfile msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	ROS_WARN_STREAM_NAMED("PanTiltMotionProfileServiceClient", "motion profile for pantilt received. Currently ignored!");
	p_query_timer.stop();
}

void PanTiltMotionProfileServiceClient_ReceiveFSM::pantilt_specification_received(JausAddress reporter, urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::ReportPanTiltSpecifications spec)
{
	ROS_WARN_STREAM_NAMED("PanTiltMotionProfileServiceClient", "specification received, but not handled -> not implemented");
}

void PanTiltMotionProfileServiceClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:manipulator:PanTiltMotionProfileService") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM_NAMED("PanTiltMotionProfileServiceClient", "unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PanTiltMotionProfileServiceClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PanTiltMotionProfileServiceClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void PanTiltMotionProfileServiceClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	// try to get the specification from robot
	if (p_remote_addr.get() != 0) {
		p_query_timer.stop();
		ROS_INFO_NAMED("PanTiltMotionProfileServiceClient", "create QUERY timer to get pantilt motion profile from %s", p_remote_addr.str().c_str());
		p_query_timer = p_nh.createTimer(ros::Duration(3.0), &PanTiltMotionProfileServiceClient_ReceiveFSM::pQueryCallback, this);
	}
}

void PanTiltMotionProfileServiceClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
}

void PanTiltMotionProfileServiceClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_mp, p_remote_addr);
	}
}




};
