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
/*!\file    plugins/tcp/internal/tNetworkServerPort.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-02-22
 *
 * \brief   Contains tNetworkServerPort
 *
 * \b tNetworkServerPort
 *
 * Proxy for server port in remote runtime environment.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tNetworkServerPort_h__
#define __plugins__tcp__internal__tNetworkServerPort_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/rpc_ports/tServerPort.h"

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
class tRemotePart;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! RPC network port
/*!
 * Proxy for server port in remote runtime environment.
 */
class tNetworkServerPort : public rpc_ports::internal::tRPCPort
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tNetworkServerPort(tRemotePart& remote_part, core::tFrameworkElement* parent, const std::string& name, rrlib::rtti::tType type, core::tFrameworkElementFlags flags);

  /*!
   * Called by TCP thread. Sends any enqueued calls.
   */
  void DoSendCalls();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Reference to remote part that this port belongs to */
  tRemotePart& remote_part;

  /*! Queue with incoming calls */
  rrlib::concurrent_containers::tQueue < tCallPointer, rrlib::concurrent_containers::tConcurrency::MULTIPLE_WRITERS,
        rrlib::concurrent_containers::tDequeueMode::ALL, false > incoming_call_queue;


  virtual void SendCall(tCallPointer && call_to_send); // TODO: mark override in gcc 4.7

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
