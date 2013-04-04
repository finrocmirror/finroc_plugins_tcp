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
/*!\file    plugins/tcp/internal/tPeerInfo.h
 *
 * \author  Max Reichardt
 * \author  Michael Arndt
 *
 * \date    2013-01-04
 *
 * \brief   Contains tPeerInfo
 *
 * \b tPeerInfo
 *
 * Info on TCP peer.
 * This is info is exchanged among different peers (and possibly combined).
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tPeerInfo_h__
#define __plugins__tcp__internal__tPeerInfo_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/protocol_definitions.h"

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
class tRemotePart;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP Peer Info
/*!
 * Info on TCP peer.
 * This is info is exchanged among different processes (and possibly combined).
 */
struct tPeerInfo : boost::noncopyable
{
  /*!
   * UUID of peer.
   *
   * A set of interconnected finroc parts maintains a list of peers.
   * To avoid duplicates in this list, the UUID of a part must be uniquely
   * defined among these connected parts.
   *
   * This UUID is not unique with respect to time:
   * If a process is started on the same system and listens on the same port, it will receive the same UUID.
   * This property can be useful - as parts can be terminated and restarted
   * and the others will attempt to reconnect automatically.
   */
  tUUID uuid;

  /*! Type of peer */
  tPeerType peer_type;

  /*!
   * Vector containing all network addresses of this peer (IPv6 and IPv4),
   * The priority is in descending order (first element is address that should be preferred)
   */
  std::vector<boost::asio::ip::address> addresses;

  /*! Are we currently connected with this peer? */
  bool connected;

  /*!
   * The number of active connections and active connecting attempts to the peer initiated by this peer
   * Typically, no new connection attempts should be made when this is not zero.
   */
  int connecting;

  /*! If we're not connected: When was the last time we were connected with this peer? */
  rrlib::time::tTimestamp last_connection;

  /*! Never forget this peer (typically true for peers that user specified) */
  bool never_forget;

  /*!
   * Pointer to tRemotePart object that represents remote part in this runtime.
   * Set when connection is established
   */
  tRemotePart* remote_part;


  tPeerInfo(tPeerType peer_type);

  /*! Host name of peer */
  std::string Hostname()
  {
    return uuid.host_name;
  }

  /*!
   * RAII class to reliably keep 'connecting' variable up to date.
   * Instantiated once for each connecting attempt.
   *
   * Increases 'connecting' when an instance of this class is created.
   * Decreases 'connecting' on deletion.
   */
  class tActiveConnect
  {
  public:

    /*!
     * \param peer_info Peer the connection attempt is made to (and whose 'connecting' variable will be updated)
     */
    tActiveConnect(tPeerInfo& peer_info);
    ~tActiveConnect();

  private:

    /*! Peer the connection attempt is made to (and whose 'connecting' variable will be updated) */
    tPeerInfo& peer_info;
  };
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
