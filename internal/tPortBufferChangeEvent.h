//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/tcp/internal/tPortBufferChangeEvent.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-02-24
 *
 * \brief   Contains tPortBufferChangeEvent
 *
 * \b tPortBufferChangeEvent
 *
 * Whenever value of an observed network port changes, this is stored in such an event.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tPortBufferChangeEvent_h__
#define __plugins__tcp__internal__tPortBufferChangeEvent_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/data_ports/tPort.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace tcp
{
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
class tNetworkPortInfo;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Port change event
/*!
 * Whenever value of an observed network port changes, this is stored in such an event.
 */
struct tPortBufferChangeEvent :
  public rrlib::buffer_pools::tBufferManagementInfo,
  public rrlib::concurrent_containers::tQueueable<rrlib::concurrent_containers::tQueueability::MOST_OPTIMIZED>,
  public rrlib::buffer_pools::tNotifyOnRecycle
{

  /*! New value of port */
  data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject> new_value;

  /*! Network port info object of network port */
  tNetworkPortInfo* network_port_info;

  /*! Change type */
  data_ports::tChangeStatus change_type;


  tPortBufferChangeEvent() :
    new_value(),
    network_port_info(NULL),
    change_type(data_ports::tChangeStatus::NO_CHANGE)
  {}

  void OnRecycle()
  {
    new_value.Reset();
  }
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
