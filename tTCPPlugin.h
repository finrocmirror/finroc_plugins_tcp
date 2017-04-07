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
/*!\file    plugins/tcp/tTCPPlugin.h
 *
 * \author  Max Reichardt
 *
 * \date    2017-03-19
 *
 * \brief   Contains tTCPPlugin
 *
 * \b tTCPPlugin
 *
 * Plugin providing full-featured network transport over TCP/IP - using the generic_protocol from finroc_plugins_network_transport.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__tTCPPlugin_h__
#define __plugins__tcp__tTCPPlugin_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/network_transport/generic_protocol/tNetworkTransportPlugin.h"

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

namespace internal
{
class tPeerImplementation;
}

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
//! Custom TCP-based network transport
/*!
 * Plugin providing full-featured network transport over TCP/IP - using the generic_protocol from finroc_plugins_network_transport.
 */
class tTCPPlugin : public network_transport::generic_protocol::tNetworkTransportPlugin
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*! Network addresses of peers to connect to */
  tParameter<std::vector<std::string>> par_connect_to;

  /*! Port that we will try to open for server. Will try the next ones if not available (SERVER and FULL only) */
  tStaticParameter<int> par_preferred_server_port;

  /*! Try the following ports, if specified port is already occupied? */
  tStaticParameter<bool> par_try_next_ports_if_occupied;

  /*! Auto-connect to all peers that become known? */
  tParameter<bool> par_auto_connect_to_all_peers;

  /*! The address that server is supposed to listen on ("::" will enable IPv6) */
  tStaticParameter<std::string> par_server_listen_address; // = "0.0.0.0";

  /* Type of peer to be created */
  tStaticParameter<tPeerType> par_peer_type;

  /*! Help for debugging: insert checks in data stream => more bandwidth */
  tStaticParameter<bool> par_debug_tcp;

  /*! Maximum number of port to try to create a server port on - if default port is occupied */
  tStaticParameter<int> par_max_ports_to_try_creating_server_port;

  /*! Minimum interval between sending data of the same port twice (express connection) */
  tParameter<rrlib::time::tDuration> par_min_update_interval_express;

  /*! Minimum interval between sending data of the same port twice (bulk connection) */
  tParameter<rrlib::time::tDuration> par_min_update_interval_bulk;

  /*! More or less the cycle time of TCP event processing loop (default 5ms) - lower results in less latency */
  tParameter<rrlib::time::tDuration> par_process_events_call_interval;

  /*! Frequency with which to call ProcessLowPriorityTasks (e.g. connecting and exchanging peer information) (default 500ms) */
  tParameter<rrlib::time::tDuration> par_process_low_priority_tasks_call_interval;


  /*!
   * Conveniently adds entry to 'par_connect_to'
   *
   * \param address Address of runtime to actively connect to
   */
  void AddRuntimeToConnectTo(const std::string& address);

  /*!
   * \return Singleton instance of TCP plugin
   */
  static tTCPPlugin& GetInstance();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class internal::tPeerImplementation;

  tTCPPlugin();

  ~tTCPPlugin();
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
