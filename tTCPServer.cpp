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
#include "plugins/tcp/tTCPPeer.h"

#include "plugins/tcp/tTCPServer.h"
#include "core/tCoreFlags.h"
#include "core/tLockOrderLevels.h"
#include "plugins/tcp/tTCPSettings.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "plugins/tcp/tTCPServerConnection.h"

namespace finroc
{
namespace tcp
{
tTCPServer::tTCPServer(int port_, bool try_next_ports_if_occupied_, tTCPPeer* peer_) :
    core::tFrameworkElement(peer_, "TCP Server", core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT, core::tLockOrderLevels::cLEAF_GROUP),
    port(port_),
    try_next_ports_if_occupied(try_next_ports_if_occupied_),
    serving(false),
    peer(peer_)
{
  tTCPSettings::InitInstance();
}

void tTCPServer::AcceptConnection(::std::shared_ptr<util::tNetSocket> s, int8 first_byte)
{
  util::tLock lock1(this);
  if (IsDeleted())
  {
    try
    {
      s->Close();
    }
    catch (const util::tIOException& e)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_WARNING, log_domain, e);
    }
    return;
  }
  try
  {
    __attribute__((unused))
    tTCPServerConnection* connection = new tTCPServerConnection(s, first_byte, this, peer);
  }
  catch (const util::tException& e)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_WARNING, log_domain, e);
  }
}

void tTCPServer::PostChildInit()
{
  ::finroc::core::tFrameworkElement::PostChildInit();
  while (true)
  {
    serving = util::tTCPConnectionHandler::AddServer(this, port);
    if (serving || (!try_next_ports_if_occupied))
    {
      break;
    }
    int next_port = port + 1;
    FINROC_LOG_STREAM(rrlib::logging::eLL_USER, log_domain, "Port ", port, " occupied - trying ", next_port);
    port++;
  }

  //AbstractPeerTracker.registerServer(networkName, name, port);
}

} // namespace finroc
} // namespace tcp

