/**
 * You received this file as part of an advanced experimental
 * robotics framework prototype ('finroc')
 *
 * Copyright (C) 2007-2010 Max Reichardt,
 *   Robotics Research Lab, University of Kaiserslautern
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include "core/portdatabase/tDataType.h"
#include "tcp/tTCPPort.h"
#include "tcp/tTCPConnection.h"
#include "core/port/net/tRemoteTypes.h"
#include "core/settings/tSetting.h"
#include "core/tRuntimeSettings.h"
#include "finroc_core_utils/tGarbageCollector.h"
#include "core/port/rpc/tAbstractCall.h"
#include "core/port/rpc/tMethodCallException.h"
#include "core/port/rpc/tSynchMethodCallLogic.h"

namespace finroc
{
namespace tcp
{
tTCPPort::tTCPPort(core::tPortCreationInfo pci, tTCPConnection* connection_) :
    core::tNetPort(pci, connection_),
    connection(connection_),
    monitored(false),
    update_interval_partner(-1)
{
  assert((connection_ != NULL));
}

int16 tTCPPort::GetUpdateIntervalForNet()
{
  int16 t = 0;
  tTCPConnection* c = connection;

  // 1. does destination have any wishes/requirements?
  if ((t = update_interval_partner) >= 0)
  {
    return t;

    // 2. any local suggestions?
  }
  else if ((t = GetPort()->GetMinNetworkUpdateIntervalForSubscription()) >= 0)
  {
    return t;

    // 3. data type default
  }
  else if ((t = GetPort()->GetDataType()->GetUpdateTime()) >= 0)
  {
    return t;

    // 4. server data type default
  }
  else if (c != NULL && (t = c->update_times.GetTime(GetPort()->GetDataType())) >= 0)
  {
    return t;
  }

  // 5. runtime default
  return static_cast<int16>(core::tRuntimeSettings::cDEFAULT_MINIMUM_NETWORK_UPDATE_TIME->Get());
}

void tTCPPort::PortChanged()
{
  tTCPConnection* c = connection;
  if (monitored && c != NULL)
  {
    c->NotifyWriter();
  }
}

void tTCPPort::PrepareDelete()
{
  SetMonitored(false);
  ::finroc::core::tNetPort::PrepareDelete();
  connection = NULL;
  util::tGarbageCollector::DeleteDeferred(this);
}

void tTCPPort::SendCall(core::tAbstractCall* mc)
{
  tTCPConnection* c = connection;
  if (c != NULL)
  {
    // we received a method/pull call that we will forward over the net
    //mc.pushCaller(getPort());
    mc->SetRemotePortHandle(this->remote_handle);
    mc->SetLocalPortHandle(GetPort()->GetHandle());
    c->SendCall(mc);
  }
  else
  {
    mc->SetExceptionStatus(core::tMethodCallException::eNO_CONNECTION);
    core::tSynchMethodCallLogic::HandleMethodReturn(mc);
    // no connection - throw exception
    //mc.setStatus(AbstractCall.CONNECTION_EXCEPTION);
    //mc.returnToCaller();
  }
}

void tTCPPort::SendCallReturn(core::tAbstractCall* mc)
{
  tTCPConnection* c = connection;
  if (c != NULL)
  {
    // we received a method/pull call that we will forward over the net
    //mc.pushCaller(getPort());
    mc->SetRemotePortHandle(this->remote_handle);
    mc->SetLocalPortHandle(GetPort()->GetHandle());
    c->SendCall(mc);
  }
  else
  {
    mc->SetExceptionStatus(core::tMethodCallException::eNO_CONNECTION);
    core::tSynchMethodCallLogic::HandleMethodReturn(mc);
    // no connection - throw exception
    //mc.setStatus(AbstractCall.CONNECTION_EXCEPTION);
    //mc.returnToCaller();
  }
}

void tTCPPort::SetMonitored(bool monitored2)
{
  tTCPConnection* c = connection;
  if (c != NULL)
  {
    if (monitored2 && !monitored)
    {
      c->monitored_ports.Add(this, false);
      monitored = true;
      c->NotifyWriter();
    }
    else if (!monitored2 && monitored)
    {
      c->monitored_ports.Remove(this);
      monitored = false;
    }
  }
  else
  {
    monitored = false;
  }
}

} // namespace finroc
} // namespace tcp

