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
/*!\file    plugins/tcp/internal/tNetworkServerPort.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-02-22
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tNetworkServerPort.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tNetworkPortInfo.h"
#include "plugins/tcp/internal/tPeerImplementation.h"
#include "plugins/tcp/internal/tRemotePart.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
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

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

class tRPCCallHandler
{
public:

  tRPCCallHandler(tNetworkServerPort& port) : port(&port)
  {}

  void operator()()
  {
    if (port->IsReady())
    {
      port->DoSendCalls();
    }
  }

private:

  tNetworkServerPort* port;
};

static core::tAbstractPortCreationInfo CreatePortCreationInfo(core::tFrameworkElement* parent, const std::string& name,
    rrlib::rtti::tType type, core::tFrameworkElementFlags flags)
{
  core::tAbstractPortCreationInfo result;
  result.parent = parent;
  result.name = name;
  result.data_type = type;
  result.flags = flags | core::tFrameworkElementFlag::ACCEPTS_DATA;
  return result;
}

tNetworkServerPort::tNetworkServerPort(tRemotePart& remote_part, core::tFrameworkElement* parent, const std::string& name, rrlib::rtti::tType type, core::tFrameworkElementFlags flags) :
  tRPCPort(CreatePortCreationInfo(parent, name, type, flags), NULL),
  remote_part(remote_part),
  incoming_call_queue()
{}

void tNetworkServerPort::DoSendCalls()
{
  rrlib::concurrent_containers::tQueueFragment<tCallPointer> waiting_calls = incoming_call_queue.DequeueAll();
  rrlib::time::tTimestamp time_now = rrlib::time::Now();
  tNetworkPortInfo* network_port_info = this->GetAnnotation<tNetworkPortInfo>();
  if (!waiting_calls.Empty())
  {
    do
    {
      tCallPointer call = waiting_calls.PopFront();
      call->SetRemotePortHandle(network_port_info->GetRemoteHandle());
      remote_part.SendCall(call, time_now);
    }
    while (!waiting_calls.Empty());
    remote_part.SendPendingMessages(time_now);
  }
}

void tNetworkServerPort::SendCall(tCallPointer && call_to_send)
{
  //assert(IsReady()); // with bad timing port can have just been prepared for deletion - however, this does not matter and is safe
  incoming_call_queue.Enqueue(call_to_send);
  remote_part.GetPeerImplementation().IOService().post(tRPCCallHandler(*this));
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
