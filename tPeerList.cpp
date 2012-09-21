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
#include "plugins/tcp/tPeerList.h"
#include "core/tRuntimeEnvironment.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "rrlib/serialization/serialization.h"

namespace finroc
{
namespace tcp
{
tPeerList::tPeerList(int server_port_, int lock_order) :
  core::tAbstractPeerTracker(lock_order),
  peers(),
  revision(0),
  server_port(server_port_)
{
  if (server_port_ > 0)
  {
    peers.push_back(util::tIPSocketAddress("localhost", server_port_));
    revision++;
  }
}

void tPeerList::AddPeer(util::tIPSocketAddress isa, bool notify_on_change)
{
  {
    rrlib::thread::tLock lock2(*this);
    if (std::find(peers.begin(), peers.end(), isa) != peers.end())
    {
      return;
    }
  }

  {
    rrlib::thread::tLock lock2(core::tRuntimeEnvironment::GetInstance()->GetRegistryLock());
    bool add = false;
    {
      rrlib::thread::tLock lock3(*this);
      add = std::find(peers.begin(), peers.end(), isa) == peers.end();
      if (add)
      {
        FINROC_LOG_PRINT(DEBUG, "received new peer: ", isa.ToString());
        peers.push_back(isa);
      }
    }

    if (add)
    {
      if (notify_on_change)
      {
        NotifyDiscovered(isa, isa.ToString());
      }
      revision++;
    }
  }
}

void tPeerList::DeserializeAddresses(rrlib::serialization::tInputStream* ci, util::tIPAddress own_address, util::tIPAddress partner_address)
{
  int size = ci->ReadInt();
  for (int i = 0; i < size; i++)
  {
    util::tIPSocketAddress ia = util::tIPSocketAddress::Deserialize(ci);
    if (ia.GetAddress().Equals(own_address) && ia.GetPort() == server_port)
    {
      // skip... because we are that
    }
    else
    {
      // replace partner's localhost entries with partnerAddress
      if (ia.GetAddress().IsLocalHost())
      {
        ia = util::tIPSocketAddress(partner_address, ia.GetPort());
      }

      AddPeer(ia, true);
    }
  }
}

void tPeerList::RemovePeer(util::tIPSocketAddress isa)
{
  // make sure: peer can only be removed, while there aren't any other connection events being processed
  std::vector<core::tAbstractPeerTracker::tListener*> listeners_copy;
  this->listeners.GetListenersCopy(listeners_copy);
  std::vector<core::tAbstractPeerTracker::tListener*> post_process;
  std::vector<util::tObject*> post_process_obj;
  {
    rrlib::thread::tLock lock2(core::tRuntimeEnvironment::GetInstance()->GetRegistryLock());
    rrlib::thread::tLock lock3(*this);
    if (std::find(peers.begin(), peers.end(), isa) != peers.end())
    {
      peers.erase(std::remove(peers.begin(), peers.end(), isa), peers.end());
      for (size_t i = 0u, n = listeners_copy.size(); i < n; i++)
      {
        ::finroc::util::tObject* o = listeners_copy[i]->NodeRemoved(isa, isa.ToString());
        if (o != NULL)
        {
          post_process.push_back(listeners_copy[i]);
          post_process_obj.push_back(o);
        }
      }
      revision++;
    }
  }

  for (size_t i = 0u, n = post_process.size(); i < n; i++)
  {
    post_process[i]->NodeRemovedPostLockProcess(post_process_obj[i]);
  }
}

void tPeerList::SerializeAddresses(rrlib::serialization::tOutputStream* co)
{
  rrlib::thread::tLock lock1(*this);
  int size = peers.size();
  co->WriteInt(size);
  for (int i = 0; i < size; i++)
  {
    peers[i].Serialize(co);
  }
}

} // namespace finroc
} // namespace tcp

