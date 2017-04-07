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
/*!\file    plugins/tcp/internal/tPeerImplementation.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 * \brief   Contains tPeerImplementation
 *
 * \b tPeerImplementation
 *
 * Implementation of different variants of TCP peer.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tPeerImplementation_h__
#define __plugins__tcp__internal__tPeerImplementation_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <set>

#include "rrlib/thread/tThread.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tPeerInfo.h"

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
class tServer;
class tConnection;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP Peer implementation
/*!
 * Implementation of different variants of TCP peer.
 */
class tPeerImplementation : public tTCPPlugin
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \param framework_element Framework element associated with peer
   * \param Options for peer creation
   */
  tPeerImplementation();

  /*! Shuts peer down */
  ~tPeerImplementation();

  /*!
   * Adds a address for this peer (if not already set)
   */
  void AddAddress(const boost::asio::ip::address& address);

  /*! Starts actively connecting to the specified network */
  void Connect();

  /*!
   * Deserialize tPeerInfo from an input stream for peer exchange
   * \param stream the input stream to deserialize from
   * \param p the tPeerInfo to deserialize into
   */
  void DeserializePeerInfo(rrlib::serialization::tInputStream& stream, tPeerInfo& p);

  /*!
   * \return Peer info about this peer
   */
  const tPeerInfo& GetPeerInfo() const
  {
    return this_peer;
  }

  /*!
   * Gets remote runtime with specified UUID.
   * If no such runtime has been registered yet, creates a new one.
   *
   * \param uuid UUID of remote runtime
   * \param peer_type Peer type
   * \param peer_name Name of peer. Will be displayed in tooling and status messages. Does not need to be unique. Typically the program/process name.
   * \param address IP address of remote runtime
   * \param never_forget Is this a remote peer to never forget?
   * \return Pointer to remote runtime
   */
  network_transport::generic_protocol::tRemoteRuntime* GetRemoteRuntime(std::shared_ptr<tConnection>& connection, const tUUID& uuid, tPeerType peer_type, const std::string& peer_name, const boost::asio::ip::address& address, bool never_forget);

  /*!
   * \return Thread id of TCP thread. Null, if TCP thread does not exist
   */
  typename rrlib::thread::tThread::tThreadId GetTCPThreadId()
  {
    return thread ? thread->GetId() : 0;
  }

  virtual void Init(rrlib::xml::tNode* config_node) override;

  /*!
   * Reference to Boost asio IO service
   */
  boost::asio::io_service& IOService()
  {
    return *io_service;
  }

  /*!
   * When there are active connections, called with high frequency (every 5 ms)
   * to process incoming events (such as port data and structure changes)
   */
  void ProcessEvents();

  /*!
   * Processes tPeerInfo received e.g. from other peers
   * \param peer_info the info to be processed
   */
  void ProcessIncomingPeerInfo(const tPeerInfo& peer_info);

  /*!
   * Processes incoming structure changes from local runtime
   */
  void ProcessLocalRuntimeStructureChanges()
  {
    tTCPPlugin::ProcessLocalRuntimeStructureChanges();
  }

  /*!
   * Called in a regular interval to do things like establishing new connections
   */
  void ProcessLowPriorityTasks();

  /*!
   * Starts running event loop (the one that call ProcessEvents() every 5ms),
   * unless it's already running
   */
  void RunEventLoop();

  /*!
   * Mark the peer list as changed.
   * This will cause the peer list to be sent to all connected peers.
   */
  void SetPeerListChanged()
  {
    peer_list_changed = true;
  }

  /*!
   * Starts server: Peer thread is instantiated and socket is opened
   */
  void StartServer();

  /*!
   * \return Unused buffer for initialization of prototype streams
   */
  rrlib::serialization::tStackMemoryBuffer<16>& UnusedInitializationBuffer()
  {
    return unused_initialization_buffer;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class tTCPPluginInstance;
  friend class tServer;
  friend class tTCPThread;
  friend class tRemotePart;

  template <bool REGULAR>
  friend struct tProcessLowPriorityTasksCaller;
  friend struct tProcessEventsCaller;

  friend struct tAddressConnectorTask;
  friend struct tConnectorTask;

  /*! Unused buffer for initialization of prototype streams (must not be destroyed before connections are) */
  rrlib::serialization::tStackMemoryBuffer<16> unused_initialization_buffer;

  /*! Vector containing all network addresses this peer should currently try to connect to */
  std::vector<std::string> connect_to;

  /*! Vector containing all network addresses this peer already connected to (should not be added to connect_to again) */
  std::set<std::string> connected_to;

  /*! Info on this peer */
  tPeerInfo this_peer;

  /*!
   * List of network peers that can be connected to
   * Each entry contains reference to tRemotePart instance as soon as a
   * connection to remote part is established.
   */
  std::vector<std::shared_ptr<tPeerInfo>> other_peers;

  /*! Revision of peer information */
  //int32_t peer_list_revision;

  /*! Set to true if peer list has changed and needs to be sent */
  bool peer_list_changed;

  /*! Primary TCP thread that does all the socket related work for this peer */
  std::shared_ptr<rrlib::thread::tThread> thread;

  /*! Boost asio IO service */
  std::shared_ptr<boost::asio::io_service> io_service;

  /*! Timers for calling ProcessLowPriorityTasks() and ProcessEvents() regularly */
  boost::asio::deadline_timer low_priority_tasks_timer, event_processing_timer;

  /*! TCP server - NULL if this is a client-only peer */
  tServer* server;

  /*!
   * Actively connect to specified network?
   * False initially; true after Connect() has been called
   */
  bool actively_connect;

  /*! True when event loop is running (ProcessEvents() is regularly called by TCP thread) */
  bool event_loop_running;


  /*!
   * Adds all the provided addresses to the specified peer info
   * (if they have not been added already)
   *
   * \param existing_peer Peer to add addresses to
   * \param addresses Addresses to check and possibly add
   */
  void AddPeerAddresses(tPeerInfo& existing_peer, const std::vector<boost::asio::ip::address>& addresses);

  /*!
   * Scans current peer list and adds missing addresses
   * (e.g. if two peers have the same host, they must both have the same IP addresses)
   */
  void InferMissingAddresses();

  virtual void OnStartServingStructure() override;

  /*!
   * Serialize tPeerInfo to an output stream for peer exchange
   * \param stream the output stream to serialize to
   * \param p the tPeerInfo to be serialized
   */
  void SerializePeerInfo(rrlib::serialization::tOutputStream& stream, const tPeerInfo& p);

  /*! Starts TCP Thread */
  void StartThread();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
