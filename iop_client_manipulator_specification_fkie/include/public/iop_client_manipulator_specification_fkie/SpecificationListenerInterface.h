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

#ifndef IOP_SPECIFICATION_LISTENER_INTERFACE_H_
#define IOP_SPECIFICATION_LISTENER_INTERFACE_H_

#include "Transport/JausAddress.h"
#include "urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient/Messages/MessageSet.h"

namespace iop
{
	class SpecificationListenerInterface
	{
		/**
		 * The interface is used by the iop_event_fkie to forward events to clients.
		 * To receive events IOP client components should include this interface.
		 */
	public:
		// ____________________________
		// === must be overridden =====
		// ============================
		virtual void specification_received(JausAddress reporter, urn_jaus_jss_manipulator_ManipulatorSpecificationServiceClient::ReportManipulatorSpecifications spec) = 0;

		virtual ~SpecificationListenerInterface(){}

	protected:
		SpecificationListenerInterface() {}
	};

};

#endif
