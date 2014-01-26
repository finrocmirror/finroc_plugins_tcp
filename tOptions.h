//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    plugins/tcp/tOptions.h
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-26
 *
 * \brief   Contains tOptions
 *
 * \b tOptions
 *
 * Options that can be set for TCP plugin.
 * To change default options from your program, get and change them
 * before the TCP plugin is created.
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__tOptions_h__
#define __plugins__tcp__tOptions_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/time/time.h"

//----------------------------------------------------------------------
// Internal includes with ""
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

/*! Mode of peer */
enum class tPeerType
{
  CLIENT_ONLY,  //!< Peer is client only
  SERVER_ONLY,  //!< Peer is server only
  FULL,         //!< Peer is client and server
  UNSPECIFIED   //!< Peer type is not specified
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP plugin options.
/*!
 * Options that can be set for TCP plugin.
 * To change default options from your program, get and change them
 * before the TCP plugin is created.
 */
struct tOptions
{
  /*! Name of peer. Will be displayed in tooling and status messages. Does not need to be unique. Typically the program/process name. */
  std::string peer_name;

  /*! Network addresses of peers to connect to */
  std::vector<std::string> connect_to;

  /*! Port that we will try to open for server. Will try the next ones if not available (SERVER and FULL only) */
  int preferred_server_port;

  /*! Try the following ports, if specified port is already occupied? */
  bool try_next_ports_if_occupied;

  /*! Auto-connect to all peers that become known? */
  bool auto_connect_to_all_peers;

  /*! The address that server is supposed to listen on ("::" will enable IPv6) */
  std::string server_listen_address; // = "0.0.0.0";

  /* Type of peer to be created */
  tPeerType peer_type;


  // Parameters from tSettings.h - On tPeer creation they will be set from these values

  /*! Maximum packets to send without acknowledgement in express connections */
  uint32_t max_not_acknowledged_packets_express;

  /*! Maximum packets to send without acknowledgement in bulk connections */
  uint32_t max_not_acknowledged_packets_bulk;

  /*! Minimum interval between sending data of the same port twice (express connection) */
  rrlib::time::tDuration min_update_interval_express;

  /*! Minimum interval between sending data of the same port twice (bulk connection) */
  rrlib::time::tDuration min_update_interval_bulk;

  /*! Critical ping threshold (if this is exceeded ports are notified of disconnect) */
  rrlib::time::tDuration critical_ping_threshold;


  tOptions() :
    peer_name("Peer name not set"),
    connect_to(),
    preferred_server_port(4444),
    try_next_ports_if_occupied(true),
    auto_connect_to_all_peers(true),
    server_listen_address("0.0.0.0"),
    peer_type(tPeerType::FULL),
    max_not_acknowledged_packets_express(10),
    max_not_acknowledged_packets_bulk(2),
    min_update_interval_express(std::chrono::milliseconds(1)),
    min_update_interval_bulk(std::chrono::milliseconds(40)),
    critical_ping_threshold(std::chrono::milliseconds(1500))
  {
  }

  /*!
   * \return Returns default options. They will be used for peer creation in standard Finroc parts.
   */
  static tOptions& GetDefaultOptions()
  {
    static tOptions options;
    return options;
  }
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
