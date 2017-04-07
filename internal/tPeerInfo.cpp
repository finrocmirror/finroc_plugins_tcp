//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/tcp/internal/tPeerInfo.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tPeerInfo.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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

tPeerInfo::tPeerInfo(tPeerType peer_type) :
  uuid(),
  peer_type(peer_type),
  addresses(),
  connecting(0),
  last_connection(rrlib::time::cNO_TIME),
  never_forget(false),
  remote_runtime(nullptr)
{
}

bool tPeerInfo::AddAddress(const boost::asio::ip::address& address)
{
  for (auto it = addresses.begin(); it != addresses.end(); ++it)
  {
    if (*it == address)
    {
      return false;
    }
  }
  addresses.push_back(address);
  return true;
}

tPeerInfo::tActiveConnect::tActiveConnect(tPeerInfo& peer_info) :
  peer_info(peer_info)
{
  peer_info.connecting++;
}

tPeerInfo::tActiveConnect::~tActiveConnect()
{
  peer_info.connecting--;
}

std::string tPeerInfo::ToString() const
{
  return (name.length() > 0) ? (name + " (" + uuid.ToString() + ")") : uuid.ToString();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
