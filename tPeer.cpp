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
/*!\file    plugins/tcp/tPeer.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/tPeer.h"

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

tPeer::tPeer(const std::string& peer_name, const std::string& network_connection, int preferred_server_port, bool try_next_ports_if_occupied,
             bool auto_connect_to_all_peers, const std::string& server_listen_address) :
  core::tFrameworkElement(&core::tRuntimeEnvironment::GetInstance(), "TCP", tFlag::NETWORK_ELEMENT),
  implementation(new internal::tPeerImplementation(*this, peer_name, internal::tPeerType::FULL, network_connection, preferred_server_port,
                 try_next_ports_if_occupied, auto_connect_to_all_peers, server_listen_address))
{
}

tPeer::tPeer(const std::string& peer_name, int preferred_server_port, bool try_next_ports_if_occupied, const std::string& server_listen_address) :
  core::tFrameworkElement(&core::tRuntimeEnvironment::GetInstance(), "TCP", tFlag::NETWORK_ELEMENT),
  implementation(new internal::tPeerImplementation(*this, peer_name, internal::tPeerType::SERVER_ONLY, "", preferred_server_port,
                 try_next_ports_if_occupied, false, server_listen_address))
{
}

tPeer::tPeer(const std::string& peer_name, const std::string& network_connection, bool auto_connect_to_all_peers) :
  core::tFrameworkElement(&core::tRuntimeEnvironment::GetInstance(), "TCP", tFlag::NETWORK_ELEMENT),
  implementation(new internal::tPeerImplementation(*this, peer_name, internal::tPeerType::CLIENT_ONLY, network_connection, -1, false, auto_connect_to_all_peers, ""))
{
}

tPeer::~tPeer()
{}

void tPeer::Connect()
{
  implementation->Connect();
}

void tPeer::PostChildInit()
{
  implementation->StartServer();
}

void tPeer::PrepareDelete()
{
  implementation.reset();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
