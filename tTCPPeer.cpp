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
#include "rrlib/finroc_core_utils/container/tSimpleList.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"

namespace finroc
{
namespace tcp
{
core::tFrameworkElementTreeFilter tTCPPeer::cGUI_FILTER(core::tCoreFlags::cSTATUS_FLAGS | core::tCoreFlags::cNETWORK_ELEMENT, core::tCoreFlags::cREADY | core::tCoreFlags::cPUBLISHED);
core::tFrameworkElementTreeFilter tTCPPeer::cDEFAULT_FILTER(core::tCoreFlags::cSTATUS_FLAGS | core::tCoreFlags::cNETWORK_ELEMENT | core::tCoreFlags::cSHARED | core::tCoreFlags::cIS_PORT, core::tCoreFlags::cREADY | core::tCoreFlags::cPUBLISHED | core::tCoreFlags::cSHARED | core::tCoreFlags::cIS_PORT);
core::tFrameworkElementTreeFilter tTCPPeer::cALL_AND_EDGE_FILTER(core::tCoreFlags::cSTATUS_FLAGS, core::tCoreFlags::cREADY | core::tCoreFlags::cPUBLISHED);

tTCPPeer::tTCPPeer(const util::tString& network_name_, core::tFrameworkElementTreeFilter filter_) :
    core::tExternalConnection("TCP", network_name_),
    mode(tTCPPeer::eCLIENT),
    server(NULL),
    network_name(network_name_),
    name(""),
    ci(this),
    filter(filter_),
    tracker(NULL),
    delete_ports_on_disconnect(filter_.IsAcceptAllFilter()),
    connections(10u, 4u)
{
  // this(networkName,"",Mode.CLIENT,-1,filter,filter.isAcceptAllFilter());

  if (IsServer())
  {
    server = new tTCPServer(-1, true, this);
  }

}

tTCPPeer::tTCPPeer(const util::tString& network_name_, const util::tString& unique_peer_name, tTCPPeer::tMode mode_, int preferred_server_port, core::tFrameworkElementTreeFilter filter_, bool delete_ports_on_disconnect_) :
    core::tExternalConnection("TCP", network_name_),
    mode(mode_),
    server(NULL),
    network_name(network_name_),
    name(unique_peer_name),
    ci(this),
    filter(filter_),
    tracker(NULL),
    delete_ports_on_disconnect(delete_ports_on_disconnect_),
    connections(10u, 4u)
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
  util::tLock lock1(this);

  {
    util::tLock lock2(tracker);

    assert((IsReady()));
    tracker->AddListener(this);

    if (same_address)
    {
      ci.Reset();
      ::finroc::core::tFrameworkElement* fe = NULL;
      while ((fe = ci.Next()) != NULL)
      {
        if (typeid(*fe) == typeid(tRemoteServer))
        {
          tRemoteServer* rs = static_cast<tRemoteServer*>(fe);
          {
            util::tLock lock6(rs);
            if (rs->IsReady() && (!rs->DeletedSoon()))
            {
              rs->Reconnect();
            }
          }
        }
      }

    }
    else
    {
      // is this an ip address?
      int idx = address.IndexOf(":");
      bool ip = false;
      if (idx > 0)
      {
        util::tString host = address.Substring(0, idx);
        util::tString port = address.Substring(idx + 1);

        ip = true;
        for (size_t i = 0u; i < port.Length(); i++)
        {
          if (!util::tCharacter::IsDigit(port.CharAt(i)))
          {
            ip = false;
          }
        }

        // we don't want to connect to ourselves
        if ((host.Equals("localhost") || host.StartsWith("127.0")) && server != NULL && util::tInteger::ParseInt(port) == server->GetPort())
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
  util::tLock lock1(this);
  if (tracker != NULL)
  {
    {
      util::tLock lock3(tracker);
      tracker->RemoveListener(this);
    }
    // now we can be sure that no new nodes will be added
  }
  //tracker.delete();

  ci.Reset();
  ::finroc::core::tFrameworkElement* fe = NULL;
  while ((fe = ci.Next()) != NULL)
  {
    if (fe->IsReady() && (typeid(*fe) == typeid(tRemoteServer)))
    {
      tRemoteServer* rs = static_cast<tRemoteServer*>(fe);
      {
        util::tLock lock4(rs);
        if (rs->IsReady() && (!rs->DeletedSoon()))
        {
          rs->TemporaryDisconnect();
        }
      }
    }
  }
}

float tTCPPeer::GetConnectionQuality()
{
  float worst = 1.0f;
  core::tFrameworkElement::tChildIterator ci(this);
  for (::finroc::core::tFrameworkElement* fe = ci.Next(); fe != NULL; fe = ci.Next())
  {
    if (fe == server || fe->IsPort())
    {
      continue;
    }
    tRemoteServer* rs = static_cast<tRemoteServer*>(fe);
    if (rs->IsReady() && (!rs->DeletedSoon()))
    {
      worst = std::min(worst, rs->GetConnectionQuality());
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
    util::tSimpleList<util::tString> add_stuff;
    core::tFrameworkElement::tChildIterator ci(this);
    for (::finroc::core::tFrameworkElement* fe = ci.Next(); fe != NULL; fe = ci.Next())
    {
      if (fe == server || (!fe->IsReady()) || fe->IsPort())
      {
        continue;
      }
      tRemoteServer* rs = static_cast<tRemoteServer*>(fe);
      if (rs->DeletedSoon())
      {
        continue;
      }
      util::tString tmp = rs->GetPartnerAddress().ToString();
      if (tmp.Equals(s))
      {
        add_stuff.Insert(0u, rs->GetPingString());
      }
      else
      {
        add_stuff.Add(rs->GetPartnerAddress().ToString() + " " + rs->GetPingString());
      }
    }
    for (size_t i = 0u; i < add_stuff.Size(); i++)
    {
      s += (i == 0) ? " (" : "; ";
      s += add_stuff.Get(i);
    }
    return s + ")";
  }
}

void tTCPPeer::NodeDiscovered(const util::tIPSocketAddress& isa, const util::tString& name_)
{
  {
    util::tLock lock2(tracker);
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
    util::tLock lock2(tracker);
    if (GetFlag(core::tCoreFlags::cDELETED))
    {
      return NULL;
    }

    // remove port & disconnect
    ci.Reset();
    for (::finroc::core::tFrameworkElement* fe = ci.Next(); fe != NULL; fe = ci.Next())
    {
      if (fe == server || fe->IsPort())
      {
        continue;
      }
      tRemoteServer* rs = static_cast<tRemoteServer*>(fe);
      if (rs->GetPartnerAddress().Equals(isa) && (!rs->DeletedSoon()) && (!rs->IsDeleted()))
      {
        rs->EarlyDeletingPreparations();
        return rs;
      }
    }
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "TCPClient warning: Node ", name_, " not found");
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
  util::tLock lock1(this);
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
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_WARNING, log_domain, e);
  }
  if (tracker != NULL)
  {
    delete tracker;
  }
}

} // namespace finroc
} // namespace tcp

