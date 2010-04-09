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
#include "tcp/tPeerList.h"

namespace finroc
{
namespace tcp
{
tPeerList::tPeerList(int server_port_) :
    peers(),
    revision(0),
    server_port(server_port_)
{
  if (server_port_ > 0)
  {
    AddPeer(util::tIPSocketAddress("localhost", server_port_), false);
  }
}

void tPeerList::AddPeer(util::tIPSocketAddress isa, bool notify_on_change)
{
  util::tLock lock1(obj_synch);
  if (!peers.Contains(isa))
  {
    util::tSystem::out.Println(util::tStringBuilder("received new peer: ") + isa.ToString());
    peers.Add(isa);
    if (notify_on_change)
    {
      NotifyDiscovered(&(isa), isa.ToString());
    }
    revision++;
  }
}

void tPeerList::DeserializeAddresses(util::tInputStreamBuffer* ci, util::tIPAddress own_address, util::tIPAddress partner_address)
{
  util::tLock lock1(obj_synch);
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
      if (ia.GetAddress().Equals(util::tIPAddress::GetLocalHost()))
      {
        ia = util::tIPSocketAddress(partner_address, ia.GetPort());
      }

      AddPeer(ia, true);
    }
  }
}

void tPeerList::RemovePeer(util::tIPSocketAddress isa)
{
  util::tLock lock1(obj_synch);
  if (peers.Contains(isa))
  {
    peers.RemoveElem(isa);
    NotifyRemoved(&(isa), isa.ToString());
    revision--;
  }
}

void tPeerList::SerializeAddresses(util::tOutputStreamBuffer* co)
{
  util::tLock lock1(obj_synch);
  int size = peers.Size();
  co->WriteInt(size);
  for (int i = 0; i < size; i++)
  {
    peers.Get(i).Serialize(co);
  }
}

} // namespace finroc
} // namespace tcp

