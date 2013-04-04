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
/*!\file    plugins/tcp/internal/tNetworkPortInfo.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-02-23
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tNetworkPortInfo.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tRuntimeEnvironment.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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

tNetworkPortInfo::tNetworkPortInfo(tRemotePart& remote_part, tHandle remote_handle, int16_t strategy, bool server_port, core::tAbstractPort& port) :
  remote_handle(remote_handle),
  server_port(server_port),
  strategy(strategy),
  current_subscription_strategy(-1),
  current_subscription_reverse_push(false),
  current_subscription_update_time(-1),
  subscription_check_pending(false),
  remote_part(remote_part),
  values_to_send(),
  deleted(false),
  last_update(rrlib::time::cNO_TIME),
  desired_encoding(rrlib::serialization::tDataEncoding::BINARY)
{
  port.AddAnnotation(*this);
  if (data_ports::IsDataFlowType(port.GetDataType()))
  {
    if (port.IsOutputPort())
    {
      values_to_send.SetMaxLength(port.GetFlag(core::tFrameworkElement::tFlag::PUSH_STRATEGY_REVERSE) ? 1 : 0);
    }
    else
    {
      values_to_send.SetMaxLength(strategy < 0 ? 0 : strategy);
    }
  }
}

void tNetworkPortInfo::AnnotatedObjectToBeDeleted()
{
  assert((core::tRuntimeEnvironment::ShuttingDown() || rrlib::thread::tThread::CurrentThreadId() == remote_part.GetPeerImplementation().GetTCPThreadId()) && "Deleting should only be done by TCP thread");
  deleted = true;
  remote_part.PortDeleted(this);
}

void tNetworkPortInfo::DoSubscriptionCheck()
{
  data_ports::common::tAbstractDataPort* port = GetAnnotated<data_ports::common::tAbstractDataPort>();
  FINROC_LOG_PRINT(DEBUG, "Checking subscription of ", port->GetQualifiedName(), " ", port->GetStrategy(), " ", port->CountIncomingConnections() + port->CountOutgoingConnections());
  subscription_check_pending.store(false); // Reset "pending" flag

  // Determine necessary subscription parameters
  // Reverse push strategy?
  bool subscription_reverse_push = false;
  if (port->IsInputPort())
  {
    for (auto it = port->IncomingConnectionsBegin(); it != port->IncomingConnectionsEnd(); ++it)
    {
      subscription_reverse_push |= static_cast<data_ports::common::tAbstractDataPort&>(*it).ReversePushStrategy();
    }
  }

  int16_t subscription_strategy = port->IsInputPort() ? 0 : port->GetStrategy();
  if (!port->IsConnected())
  {
    subscription_strategy = -1;
  }

  int16_t subscription_update_time = -1; // TODO: GetUpdateIntervalForNet();

  // Possibly update subscription
  if (subscription_strategy == -1 && current_subscription_strategy > -1)    // disconnect
  {
    std::shared_ptr<tConnection> management_connection = remote_part.GetManagementConnection();
    if (management_connection)
    {
      tUnsubscribeMessage::Serialize(true, management_connection->CurrentWriteStream(), remote_handle);
      current_subscription_strategy = -1;
      current_subscription_reverse_push = false;
      current_subscription_update_time = -1;
    }
  }
  else if (subscription_strategy == -1)
  {
    // still disconnected
  }
  else if (subscription_strategy != current_subscription_strategy || subscription_update_time != current_subscription_update_time ||
           subscription_reverse_push != current_subscription_reverse_push)
  {
    std::shared_ptr<tConnection> management_connection = remote_part.GetManagementConnection();
    if (management_connection)
    {
      // Update subscription
      FINROC_LOG_PRINT(DEBUG, "Subscribing");
      tSubscribeMessage::Serialize(true, management_connection->CurrentWriteStream(), remote_handle, subscription_strategy,
                                   subscription_reverse_push, subscription_update_time, port->GetHandle(), rrlib::serialization::tDataEncoding::BINARY);
      current_subscription_strategy = subscription_strategy;
      current_subscription_update_time = subscription_update_time;
      current_subscription_reverse_push = subscription_reverse_push;
    }
  }
}

void tNetworkPortInfo::SetServerSideSubscriptionData(int16_t strategy, bool reverse_push, int16_t update_time, rrlib::serialization::tDataEncoding encoding)
{
  this->strategy = strategy;
  this->current_subscription_strategy = strategy;
  this->current_subscription_reverse_push = reverse_push;
  this->current_subscription_update_time = update_time;
  this->desired_encoding = encoding;

  core::tAbstractPort* port = GetAnnotated<core::tAbstractPort>();
  if (data_ports::IsDataFlowType(port->GetDataType()))
  {
    if (port->IsOutputPort())
    {
      values_to_send.SetMaxLength(reverse_push ? 1 : 0);
    }
    else
    {
      values_to_send.SetMaxLength(strategy < 0 ? 0 : strategy);
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
