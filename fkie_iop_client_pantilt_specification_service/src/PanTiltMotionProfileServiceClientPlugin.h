/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#ifndef PANTILTMOTIONPROFILESERVICECLIENTPLUGIN_H
#define PANTILTMOTIONPROFILESERVICECLIENTPLUGIN_H

#include "urn_jaus_jss_manipulator_PanTiltMotionProfileServiceClient/PanTiltMotionProfileServiceClientService.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClientService.h"
#include "urn_jaus_jss_core_EventsClient/EventsClientService.h"
#include "urn_jaus_jss_core_Transport/TransportService.h"

#include <fkie_iop_component/iop_plugin_interface.h>

namespace iop
{

class DllExport PanTiltMotionProfileServiceClientPlugin : public PluginInterface
{
public:
	PanTiltMotionProfileServiceClientPlugin();

	JTS::Service* get_service();
	void create_service(JTS::JausRouter* jaus_router);

protected:
	urn_jaus_jss_manipulator_PanTiltMotionProfileServiceClient::PanTiltMotionProfileServiceClientService *p_my_service;
	urn_jaus_jss_core_AccessControlClient::AccessControlClientService *p_base_service;
	urn_jaus_jss_core_EventsClient::EventsClientService *p_events_service;
	urn_jaus_jss_core_Transport::TransportService *p_transport_service;

};

};

#endif
