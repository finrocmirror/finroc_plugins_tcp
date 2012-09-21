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
#include "core/tCoreFlags.h"
#include "plugins/tcp/tTCPServer.h"
#include "plugins/tcp/tPeerList.h"
#include "plugins/tcp/tRemoteServer.h"
#include "rrlib/finroc_core_utils/net/tIPSocketAddress.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"

namespace finroc
{
namespace tcp
{
core::tFrameworkElementFilter tTCPPeer::cGUI_FILTER(core::tCoreFlags::cSTATUS_FLAGS | core::tCoreFlags::cNETWORK_ELEMENT, core::tCoreFlags::cREADY | core::tCoreFlags::cPUBLISHED, false);
core::tFrameworkElementFilter tTCPPeer::cDEFAULT_FILTER(core::tCoreFlags::cSTATUS_FLAGS | core::tCoreFlags::cNETWORK_ELEMENT | core::tCoreFlags::cSHARED | core::tCoreFlags::cIS_PORT, core::tCoreFlags::cREADY | core::tCoreFlags::cPUBLISHED | core::tCoreFlags::cSHARED | core::tCoreFlags::cIS_PORT, false);
core::tFrameworkElementFilter tTCPPeer::cALL_AND_EDGE_FILTER(core::tCoreFlags::cSTATUS_FLAGS, core::tCoreFlags::cREADY | core::tCoreFlags::cPUBLISHED, true);

tTCPPeer::tTCPPeer(const util::tString& network_name_, core::tFrameworkElementFilter filter_) :
  core::tExternalConnection("TCP", network_name_),
  mode(eCLIENT),
  server(NULL),
  network_name(network_name_),
  name(""),
  filter(filter_),
  tracker(NULL),
  delete_ports_on_disconnect(filter_.IsAcceptAllFilter()),
  connections(10)
{
  // this(networkName,"",Mode.CLIENT,-1,filter,filter.isAcceptAllFilter());

  if (IsServer())
  {
    server = new tTCPServer(-1, true, this);
  }

}

tTCPPeer::tTCPPeer(const util::tString& network_name_, const util::tString& unique_peer_name, tTCPPeer::tMode mode_, int preferred_server_port, core::tFrameworkElementFilter filter_, bool delete_ports_on_disconnect_) :
  core::tExternalConnection("TCP", network_name_),
  mode(mode_),
  server(NULL),
  network_name(network_name_),
  name(unique_peer_name),
  filter(filter_),
  tracker(NULL),
  delete_ports_on_disconnect(delete_ports_on_disconnect_),
  connections(10)
{
  if (IsServer())
  {
    server = new tTCPServer(preferred_server_port, true, this);
  }

}

void tTCPPeer::Connect()
{
  assert((IsReady()));
  ConnectImpl(network_name, false);
  PostConnect(network_name);
}

void tTCPPeer::ConnectImpl(const util::tString& address, bool same_address)
{
  tLock lock1(*this);

  {
    tLock lock2(*tracker);

    assert((IsReady()));
    tracker->AddListener(*this);

    if (same_address)
    {
      for (auto it = ChildrenBegin(); it != ChildrenEnd(); ++it)
      {
        if (it->IsReady() && typeid(*it) == typeid(tRemoteServer))
        {
          tRemoteServer& rs = static_cast<tRemoteServer&>(*it);
          tLock lock6(rs);
          if (rs.IsReady() && (!rs.DeletedSoon()))
          {
            rs.Reconnect();
          }
        }
      }

    }
    else
    {
      // is this an ip address?
      size_t idx = address.find(':');
      bool ip = false;
      if (idx != std::string::npos)
      {
        util::tString host = address.substr(0, idx);
        util::tString port = address.substr(idx + 1);

        ip = true;
        for (size_t i = 0u; i < port.length(); i++)
        {
          if (!isdigit(port[i]))
          {
            ip = false;
          }
        }

        // we don't want to connect to ourselves
        if ((boost::equals(host, "localhost") || boost::starts_with(host, "127.0")) && server != NULL && util::tInteger::ParseInt(port) == server->GetPort())
        {
          return;
        }

        if (ip)
        {
          util::tIPSocketAddress isa(host, util::tInteger::ParseInt(port));
          tracker->AddPeer(isa, false);
          tRemoteServer* rs = new tRemoteServer(isa, address, this, filter, this);
          rs->Init();
          return;
        }
      }
    }
  }

  //this.setDescription("tcp_" + address);
  //  tracker = new PeerTracker(address, this);
  //  tracker = new FixedPeerList();
}

void tTCPPeer::DisconnectImpl()
{
  tLock lock1(*this);
  if (tracker != NULL)
  {
    tLock lock3(*tracker);
    tracker->RemoveListener(*this);
    // now we can be sure that no new nodes will be added
  }
  //tracker.delete();

  for (auto it = ChildrenBegin(); it != ChildrenEnd(); ++it)
  {
    if (it->IsReady() && (typeid(*it) == typeid(tRemoteServer)))
    {
      tRemoteServer& rs = static_cast<tRemoteServer&>(*it);
      tLock lock4(rs);
      if (rs.IsReady() && (!rs.DeletedSoon()))
      {
        rs.TemporaryDisconnect();
      }
    }
  }
}

float tTCPPeer::GetConnectionQuality()
{
  float worst = 1.0f;
  for (auto it = ChildrenBegin(); it != ChildrenEnd(); ++it)
  {
    if (&(*it) == server || it->IsPort())
    {
      continue;
    }
    tRemoteServer& rs = static_cast<tRemoteServer&>(*it);
    if (rs.IsReady() && (!rs.DeletedSoon()))
    {
      worst = std::min(worst, rs.GetConnectionQuality());
    }
  }
  return worst;
}

util::tString tTCPPeer::GetStatus(bool detailed)
{
  util::tString s = ::finroc::core::tExternalConnection::GetConnectionAddress();
  if (!detailed)
  {
    return s;
  }
  else
  {
    std::vector<util::tString> add_stuff;
    for (auto it = ChildrenBegin(); it != ChildrenEnd(); ++it)
    {
      if (&(*it) == server || (!it->IsReady()) || it->IsPort())
      {
        continue;
      }
      tRemoteServer& rs = static_cast<tRemoteServer&>(*it);
      if (rs.DeletedSoon())
      {
        continue;
      }
      util::tString tmp = rs.GetPartnerAddress().ToString();
      if (boost::equals(tmp, s))
      {
        add_stuff.insert(add_stuff.begin(), rs.GetPingString());
      }
      else
      {
        add_stuff.push_back(rs.GetPartnerAddress().ToString() + " " + rs.GetPingString());
      }
    }
    for (size_t i = 0u; i < add_stuff.size(); i++)
    {
      s += (i == 0) ? " (" : "; ";
      s += add_stuff[i];
    }
    return s + ")";
  }
}

void tTCPPeer::NodeDiscovered(const util::tIPSocketAddress& isa, const util::tString& name_)
{
  {
    tLock lock2(*tracker);
    if (GetFlag(core::tCoreFlags::cDELETED))
    {
      return;
    }

    // add port & connect
    tRemoteServer* rs = new tRemoteServer(isa, name_, this, filter, this);
    rs->Init();
  }
}

::finroc::util::tObject* tTCPPeer::NodeRemoved(const util::tIPSocketAddress& isa, const util::tString& name_)
{
  {
    tLock lock2(*tracker);
    if (GetFlag(core::tCoreFlags::cDELETED))
    {
      return NULL;
    }

    // remove port & disconnect
    for (auto it = ChildrenBegin(); it != ChildrenEnd(); ++it)
    {
      if (&(*it) == server || it->IsPort())
      {
        continue;
      }
      tRemoteServer& rs = static_cast<tRemoteServer&>(*it);
      if (rs.GetPartnerAddress().Equals(isa) && (!rs.DeletedSoon()) && (!rs.IsDeleted()))
      {
        rs.EarlyDeletingPreparations();
        return &rs;
      }
    }
    FINROC_LOG_PRINT(WARNING, "TCPClient warning: Node ", name_, " not found");
    return NULL;
  }
}

void tTCPPeer::NodeRemovedPostLockProcess(util::tObject* obj)
{
  (static_cast<tRemoteServer*>(obj))->ManagedDelete();
}

void tTCPPeer::NotifyAllWriters()
{
  util::tArrayWrapper<tTCPConnection*>* it = connections.GetIterable();
  for (int i = 0, n = it->Size(); i < n; i++)
  {
    tTCPConnection* tc = it->Get(i);
    if (tc != NULL)
    {
      tc->NotifyWriter();
    }
  }
}

void tTCPPeer::PostChildInit()
{
  tracker = new tPeerList(IsServer() ? server->GetPort() : -1, GetLockOrder() + 1);
  if (IsServer())
  {
    tracker->RegisterServer(network_name, name, server->GetPort());
  }
}

void tTCPPeer::PrepareDelete()
{
  tLock lock1(*this);
  if (IsServer() && tracker != NULL)
  {
    tracker->UnregisterServer(network_name, name);
  }
  try
  {
    Disconnect();
  }
  catch (const util::tException& e)
  {
    FINROC_LOG_PRINT(DEBUG_WARNING, e);
  }
  if (tracker != NULL)
  {
    delete tracker;
  }
}

} // namespace finroc
} // namespace tcp

