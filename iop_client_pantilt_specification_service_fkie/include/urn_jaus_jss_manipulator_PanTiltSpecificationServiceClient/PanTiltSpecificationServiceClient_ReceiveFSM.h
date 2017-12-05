

#ifndef PANTILTSPECIFICATIONSERVICECLIENT_RECEIVEFSM_H
#define PANTILTSPECIFICATIONSERVICECLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/Messages/MessageSet.h"
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_client_pantilt_specification_service_fkie/PanTiltSpecificationListenerInterface.h>


#include "PanTiltSpecificationServiceClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient
{

class DllExport PanTiltSpecificationServiceClient_ReceiveFSM :
public JTS::StateMachine,
public iop::ocu::SlaveHandlerInterface
{
public:
	PanTiltSpecificationServiceClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM);
	virtual ~PanTiltSpecificationServiceClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportPanTiltSpecificationsAction(ReportPanTiltSpecifications msg, Receive::Body::ReceiveRec transportData);


	void add_listener(iop::PanTiltSpecificationListenerInterface *listener);
	/// Guard Methods

	std::pair<std::string, std::string> getJointNames();

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);


	PanTiltSpecificationServiceClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;

	std::vector<iop::PanTiltSpecificationListenerInterface*> p_spec_listener;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	tf2_ros::StaticTransformBroadcaster p_static_broadcaster;
	geometry_msgs::TransformStamped p_static_transformStamped;
	JausAddress p_remote_addr;
	bool p_has_access;

	QueryPanTiltSpecifications p_query_spec;
	std::string p_joint1_name;
	std::string p_joint2_name;
	std::string p_tf_frame_robot;
	std::string p_tf_frame_pantilt;

	void pQueryCallback(const ros::TimerEvent& event);
	void pNotifyListeners(JausAddress reporter, ReportPanTiltSpecifications spec);
};

};

#endif // PANTILTSPECIFICATIONSERVICECLIENT_RECEIVEFSM_H
