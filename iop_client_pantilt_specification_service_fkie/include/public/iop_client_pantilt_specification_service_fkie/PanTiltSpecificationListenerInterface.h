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

#ifndef IOP_PANTILT_SPECIFICATION_LISTENER_INTERFACE_H_
#define IOP_PANTILT_SPECIFICATION_LISTENER_INTERFACE_H_

#include "Transport/JausAddress.h"
#include "urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient/Messages/MessageSet.h"

namespace iop
{
	class PanTiltSpecificationListenerInterface
	{
		/**
		 * You have to implement this interface to receive notifications about received specifications
		 */
	public:
		// ____________________________
		// === must be overridden =====
		// ============================
		virtual void pantilt_specification_received(JausAddress reporter, urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient::ReportPanTiltSpecifications spec) = 0;

		virtual ~PanTiltSpecificationListenerInterface(){}

	protected:
		PanTiltSpecificationListenerInterface() {}
	};

};

#endif
