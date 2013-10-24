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
/*!\file    plugins/tcp/tPeer.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 * \brief   Contains tPeer
 *
 * \b tPeer
 *
 * A TCP Peer contains a TCP Client and a TCP Server.
 * It is a single peer in a Peer2Peer network.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__tPeer_h__
#define __plugins__tcp__tPeer_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tPeerImplementation.h"

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
// Class declaration
//----------------------------------------------------------------------
//! TCP Peer
/*!
 * A TCP Peer contains a TCP Client and a TCP Server.
 * It is a single peer in a Peer2Peer network.
 */
class tPeer : public core::tFrameworkElement
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * Creates full peer with client and server
   *
   * \param peer_name Name of peer. Will be displayed in tooling and status messages. Does not need to be unique. Typically the program/process name.
   * \param network_connection Name of network that peer belongs to OR network address of one peer that belongs to P2P network
   * \param preferred_server_port Port that we will try to open for server. Will try the next ones if not available (SERVER and FULL only)
   * \param try_next_ports_if_occupied Try the following ports, if specified port is already occupied?
   * \param auto_connect_to_all_peers Auto-connect to all peers that become known?
   * \param server_listen_address The address that server is supposed to listen on ("::" will enable IPv6)
   */
  tPeer(const std::string& peer_name, const std::string& network_connection, int preferred_server_port, bool try_next_ports_if_occupied,
        bool auto_connect_to_all_peers, const std::string& server_listen_address = "0.0.0.0");

  /*!
   * Creates server-only peer
   *
   * \param peer_name Name of peer. Will be displayed in tooling and status messages. Does not need to be unique. Typically the program/process name.
   * \param preferred_server_port Port that we will try to open for server. Will try the next ones if not available (SERVER and FULL only)
   * \param try_next_ports_if_occupied Try the following ports, if specified port is already occupied?
   * \param server_listen_address The address that server is supposed to listen on ("::" will enable IPv6)
   */
  tPeer(const std::string& peer_name, int preferred_server_port, bool try_next_ports_if_occupied, const std::string& server_listen_address = "0.0.0.0");

  /*!
   * Creates client-only peer
   *
   * \param peer_name Name of peer. Will be displayed in tooling and status messages. Does not need to be unique. Typically the program/process name.
   * \param network_connection Name of network that peer belongs to OR network address of one peer that belongs to P2P network
   * \param auto_connect_to_all_peers Auto-connect to all peers that become known?
   */
  tPeer(const std::string& peer_name, const std::string& network_connection, bool auto_connect_to_all_peers);


  /*! Starts actively connecting to the specified network */
  void Connect();

  /*!
   * Starts serving connections for structure clients
   * (typically tools such as finstruct)
   *
   * Should be called _after_ the application has been constructed
   */
  void StartServingStructure()
  {
    implementation->StartServingStructure();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Peer implementation */
  std::unique_ptr<internal::tPeerImplementation> implementation;


  virtual ~tPeer();

  virtual void PostChildInit(); // TODO: mark override in gcc 4.7

  virtual void PrepareDelete(); // TODO: mark override in gcc 4.7
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
