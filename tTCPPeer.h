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

#ifndef plugins__tcp__tTCPPeer_h__
#define plugins__tcp__tTCPPeer_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "plugins/tcp/tTCPConnection.h"
#include "core/tFrameworkElement.h"
#include "core/port/net/tFrameworkElementFilter.h"
#include "core/plugin/tExternalConnection.h"
#include "core/port/net/tAbstractPeerTracker.h"

namespace finroc
{
namespace util
{
class tIPSocketAddress;
} // namespace finroc
} // namespace util

namespace finroc
{
namespace tcp
{
class tTCPServer;
class tPeerList;

/*!
 * \author Max Reichardt
 *
 * A TCP Peer contains a TCP Client and a TCP Server.
 * It is a single peer in a Peer2Peer network.
 *
 * The current implementation is quite crude (many connections and threads).
 * TODO: improve this (client and server should use the same TCPConnections
 * to communicate with another peer).
 */
class tTCPPeer : public core::tExternalConnection, public core::tAbstractPeerTracker::tListener
{
public:

  enum tMode { eFULL, eSERVER, eCLIENT };

private:

  /*! Mode (see enum above) */
  tTCPPeer::tMode mode;

  /*! TCPServer - if this peer contains a server */
  tTCPServer* server;

  /*! Name of network server belongs to */
  util::tString network_name;

  /*! Unique Name of peer in network (empty string, if it has no unique name) */
  util::tString name;

  /*! Filter that specifies which elements in remote runtime environment we're interested in */
  core::tFrameworkElementFilter filter;

  /*! Peer tracker that we use for discovering network nodes */
  tPeerList* tracker;

  /*! Delete all ports when client disconnects? */
  bool delete_ports_on_disconnect;

public:

  /*! TreeFilter for different applications */
  static core::tFrameworkElementFilter cGUI_FILTER;

  static core::tFrameworkElementFilter cDEFAULT_FILTER;

  static core::tFrameworkElementFilter cALL_AND_EDGE_FILTER;

  /*! All active connections connected to this peer */
  util::tSafeConcurrentlyIterableList<tTCPConnection*, rrlib::thread::tOrderedMutex> connections;

protected:

  virtual void ConnectImpl(const util::tString& address, bool same_address);

  virtual void DisconnectImpl();

  virtual void PrepareDelete();

public:

  /*!
   * Constructor for client connections
   *
   * \param network_name Name of network that peer belongs to OR network address of one peer that belongs to P2P network
   * \param filter Filter that specifies which elements in remote runtime environment we're interested in. (CLIENT and FULL only)
   */
  tTCPPeer(const util::tString& network_name, core::tFrameworkElementFilter filter_);

  /*!
   * \param network_name Name of network that peer belongs to OR network address of one peer that belongs to P2P network
   * \param unique_peer_name Unique Name of TCP server in network (provide empty string, if its not practical to provide unique one)
   * \param mode Mode (see enum above)
   * \param preferred_server_port Port that we will try to open for server. Will try the next ones if not available. (SERVER and FULL only)
   * \param filter Filter that specifies which elements in remote runtime environment we're interested in. (CLIENT and FULL only)
   * \param delete_ports_on_disconnect Delete all ports when client disconnects?
   */
  tTCPPeer(const util::tString& network_name, const util::tString& unique_peer_name, tTCPPeer::tMode mode, int preferred_server_port, core::tFrameworkElementFilter filter, bool delete_ports_on_disconnect);

  /*!
   * \param connection Active connection
   */
  inline void AddConnection(tTCPConnection* connection)
  {
    connections.Add(connection, false);
  }

  /*! Start connecting
   * @throws Exception */
  void Connect();

  /*!
   * \return Delete all ports when client disconnects?
   */
  inline bool DeletePortsOnDisconnect()
  {
    return delete_ports_on_disconnect;
  }

  virtual float GetConnectionQuality();

  /*!
   * \return Peer's list of other peers
   */
  inline tPeerList* GetPeerList()
  {
    return tracker;
  }

  virtual util::tString GetStatus(bool detailed);

  /*!
   * \return Is this a connection/client used for administration?
   */
  inline bool IsAdminConnection()
  {
    return filter.IsAcceptAllFilter();
  }

  /*!
   * \return Does peer act as a client?
   */
  inline bool IsClient()
  {
    return mode == eCLIENT || mode == eFULL;
  }

  /*!
   * \return Does peer provide a server for connecting?
   */
  inline bool IsServer()
  {
    return mode == eSERVER || mode == eFULL;
  }

  virtual void NodeDiscovered(const util::tIPSocketAddress& isa, const util::tString& name_);

  virtual ::finroc::util::tObject* NodeRemoved(const util::tIPSocketAddress& isa, const util::tString& name_);

  virtual void NodeRemovedPostLockProcess(util::tObject* obj);

  /*!
   *  Notifies writers of all active connections connected to this peer
   */
  void NotifyAllWriters();

  virtual void PostChildInit();

  /*!
   * \param connection Active connection
   */
  inline void RemoveConnection(tTCPConnection* connection)
  {
    connections.Remove(connection);
  }

};

} // namespace finroc
} // namespace tcp

#endif // plugins__tcp__tTCPPeer_h__
