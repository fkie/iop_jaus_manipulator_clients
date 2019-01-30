

#ifndef PANTILTJOINTPOSITIONSENSORCLIENT_RECEIVEFSM_H
#define PANTILTJOINTPOSITIONSENSORCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_manipulator_PanTiltJointPositionSensorClient/Messages/MessageSet.h"
#include "urn_jaus_jss_manipulator_PanTiltJointPositionSensorClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <boost/thread/recursive_mutex.hpp>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/PanTiltSpecificationServiceClientService.h"

#include "PanTiltJointPositionSensorClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_manipulator_PanTiltJointPositionSensorClient
{

class DllExport PanTiltJointPositionSensorClient_ReceiveFSM :
public JTS::StateMachine,
public iop::ocu::SlaveHandlerInterface,
public iop::EventHandlerInterface
{
public:
	PanTiltJointPositionSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM);
	virtual ~PanTiltJointPositionSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportPanTiltJointPositionsAction(ReportPanTiltJointPositions msg, Receive::Body::ReceiveRec transportData);


	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);


	PanTiltJointPositionSensorClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;

	urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::PanTiltSpecificationServiceClientService* pPtSpecService;
	QueryPanTiltJointPositions p_query_pos;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	std::string p_joint1_name;
	std::string p_joint2_name;
	ros::Publisher p_pub_pos_joints;
	ros::Publisher p_pub_pos_pan;
	ros::Publisher p_pub_pos_tilt;
	ros::Publisher p_pub_pos_pan32;
	ros::Publisher p_pub_pos_tilt32;
	ros::Publisher p_sub_pos_stamped;
	bool p_use_posestamped;
	std::string p_tf_frame_pantilt;

	JausAddress p_remote_addr;
	bool p_has_access;
	bool p_by_query;
	double p_hz;
	void pQueryCallback(const ros::TimerEvent& event);
	void pUpdatePosition(double pan, double tilt);

};

};

#endif // PANTILTJOINTPOSITIONSENSORCLIENT_RECEIVEFSM_H
