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
#include <map>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/deadline_timer.hpp>
#include "rrlib/thread/tThread.h"
#include "core/tFrameworkElement.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/common/tFrameworkElementInfo.h"
#include "plugins/tcp/common/tRemoteTypes.h"
//#include "plugins/tcp/internal/tNetworkPortInfo.h"
#include "plugins/tcp/internal/tPeerInfo.h"
#include "plugins/tcp/internal/tPortBufferChangeEvent.h"
#include "plugins/tcp/internal/tSerializedStructureChange.h"

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP Peer implementation
/*!
 * Implementation of different variants of TCP peer.
 */
class tPeerImplementation : public core::tRuntimeListener
{

  typedef rrlib::buffer_pools::tBufferPool < tPortBufferChangeEvent, rrlib::concurrent_containers::tConcurrency::MULTIPLE_READERS,
          rrlib::buffer_pools::management::QueueBased, rrlib::buffer_pools::deleting::ComplainOnMissingBuffers,
          rrlib::buffer_pools::recycling::UseOwnerStorageInBuffer > tPortBufferChangeEventPool;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename tPortBufferChangeEventPool::tPointer tChangeEventPointer;

  /*!
   * \param framework_element Framework element associated with peer
   * \param peer_name Name of peer. Will be displayed in tooling and status messages. Does not need to be unique. Typically the program/process name.
   * \param peer_type Type of peer to be created
   * \param network_connection Name of network that peer belongs to OR network address of one peer that belongs to P2P network
   * \param preferred_server_port Port that we will try to open for server. Will try the next ones if not available (SERVER and FULL only)
   * \param try_next_ports_if_occupied Try the following ports, if specified port is already occupied?
   * \param auto_connect_to_all_peers Auto-connect to all peers that become known?
   * \param server_listen_address The address that server is supposed to listen on ("::" will enable IPv6)
   */
  tPeerImplementation(core::tFrameworkElement& framework_element, const std::string& peer_name, tPeerType peer_type, const std::string& network_connection,
                      int preferred_server_port, bool try_next_ports_if_occupied, bool auto_connect_to_all_peers, const std::string& server_listen_address);

  /*! Shuts peer down */
  ~tPeerImplementation();

  /*!
   * Adds a address for this peer (if not already set)
   */
  void AddAddress(const boost::asio::ip::address& address);

  /*! Starts actively connecting to the specified network */
  void Connect();

  /*!
   * \return Peer info about this peer
   */
  const tPeerInfo& GetPeerInfo() const
  {
    return this_peer;
  }

  /*!
   * Gets remote part with specified UUID.
   * If no such part has been registered yet, creates a new one.
   *
   * \param uuid UUID of remote part
   * \param peer_type Peer type
   * \param peer_name Name of peer. Will be displayed in tooling and status messages. Does not need to be unique. Typically the program/process name.
   * \param address IP address of remote part
   * \param never_forget Is this a remote peer to never forget?
   * \return Pointer to remote part
   */
  tRemotePart* GetRemotePart(const tUUID& uuid, tPeerType peer_type, const std::string& peer_name, const boost::asio::ip::address& address, bool never_forget);

  /*!
   * \return Thread id of TCP thread. Null, if TCP thread does not exist
   */
  typename rrlib::thread::tThread::tThreadId GetTCPThreadId()
  {
    return thread ? thread->GetId() : 0;
  }

  /*!
   * Reference to Boost asio IO service
   */
  boost::asio::io_service& IOService()
  {
    return io_service;
  }

  void PortChanged(data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject>& value, tNetworkPortInfo* info, data_ports::tChangeContext& change_context)
  {
    tChangeEventPointer event_buffer = port_buffer_change_event_buffers.GetUnusedBuffer();
    if (!event_buffer)
    {
      event_buffer = port_buffer_change_event_buffers.AddBuffer(std::unique_ptr<tPortBufferChangeEvent>(new tPortBufferChangeEvent()));
    }
    event_buffer->new_value = std::move(value);
    event_buffer->network_port_info = info;
    event_buffer->change_type = change_context.ChangeType();
    incoming_port_buffer_changes.Enqueue(event_buffer);
  }

  /*!
   * When there are active connections, called with high frequency (every 5 ms)
   * to process incoming events (such as port data and structure changes)
   */
  void ProcessEvents();

  /*!
   * Called in a regular interval to do things like establishing new connections
   */
  void ProcessLowPriorityTasks();

  /*!
   * Processes all enqueued runtime change events in TCP thread
   * and distributes them to all connections that are interested.
   */
  void ProcessRuntimeChangeEvents();

  /*!
   * Starts running event loop (the one that call ProcessEvents() every 5ms),
   * unless it's already running
   */
  void RunEventLoop();

  /*!
   * Serializes shared ports and returns that in memory buffer
   *
   * \param connection_type_encoder Type encoder object of connection to serialize shared ports for
   */
  rrlib::serialization::tMemoryBuffer SerializeSharedPorts(common::tRemoteTypes& connection_type_encoder);

  /*!
   * \return True as soon as peer also serves clients interested in complete application structure
   */
  bool ServesStructure() const
  {
    return serve_structure.load();
  }

  /*!
   * Starts server: Peer thread is instantiated and socket is opened
   */
  void StartServer();

  void StartServingStructure()
  {
    serve_structure.store(true);
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class tServer;
  friend class tTCPThread;

  template <bool REGULAR>
  friend struct tProcessLowPriorityTasksCaller;
  friend struct tProcessEventsCaller;

  friend struct tAddressConnectorTask;

  /*! Framework element associated with server */
  core::tFrameworkElement& framework_element;

  /*! Name of network that peer belongs to OR network address of one peer that belongs to P2P network */
  std::string network_connection;

  /*! Vector containing all network addresses this peer should try to connect to */
  std::vector<boost::asio::ip::tcp::endpoint> connect_to;

  /*! Info on this peer */
  tPeerInfo this_peer;

  /*!
   * List of network peers that can be connected to
   * Each entry contains reference to tRemotePart instance as soon as a
   * connection to remote part is established.
   */
  std::vector<std::unique_ptr<tPeerInfo>> other_peers;

  /*! Revision of peer information */
  int32_t peer_list_revision;

  /*! Primary TCP thread that does all the socket related work for this peer */
  std::shared_ptr<rrlib::thread::tThread> thread;

  /*! Boost asio IO service */
  boost::asio::io_service io_service;

  /*! Timers for calling ProcessLowPriorityTasks() and ProcessEvents() regularly */
  boost::asio::deadline_timer low_priority_tasks_timer, event_processing_timer;

  /*! TCP server - NULL if this is a client-only peer */
  tServer* server;

  /*! Cached info on ports shared by this peer */
  std::map<core::tFrameworkElement::tHandle, rrlib::serialization::tFixedBuffer> shared_ports;

  /*! Mutex for 'shared_ports' access */
  rrlib::thread::tMutex shared_ports_mutex;

  /*!
   * True as soon as peer also serves clients interested in complete application structure.
   * This may be enabled later, as this can cause quite a lot of overhead if done during
   * construction and startup of large (MCA2) applications.
   */
  std::atomic<bool> serve_structure;

  /*!
   * Concurrent queue with incoming structure changes.
   * Queue is filled when runtime changes occur (structure mutex is acquired by this thread).
   * Queue is processed when TCP thread calls ProcessIncomingStructureChanges()
   */
  rrlib::concurrent_containers::tQueue < std::unique_ptr<tSerializedStructureChange>, rrlib::concurrent_containers::tConcurrency::SINGLE_READER_AND_WRITER,
        rrlib::concurrent_containers::tDequeueMode::ALL > incoming_structure_changes;

  /*!
   * Actively connect to specified network?
   * False initially; true after Connect() has been called
   */
  bool actively_connect;

  /*! Ports to check subscriptions for */
  std::vector<tFrameworkElementHandle> pending_subscription_checks;

  /*! Mutex for 'pending_subscription_checks' access */
  rrlib::thread::tMutex pending_subscription_checks_mutex;

  /*! Copy for TCP thread */
  std::vector<tFrameworkElementHandle> pending_subscription_checks_copy;

  /*! True when event loop is running (ProcessEvents() is regularly called by TCP thread) */
  bool event_loop_running;

  /*! Concurrent queue with incoming port value changes */
  rrlib::concurrent_containers::tQueue < tChangeEventPointer, rrlib::concurrent_containers::tConcurrency::MULTIPLE_WRITERS,
        rrlib::concurrent_containers::tDequeueMode::ALL > incoming_port_buffer_changes;

  /*! Buffer pool with port value change event buffers */
  tPortBufferChangeEventPool port_buffer_change_event_buffers;

  /*! List with deleted RPC ports - so that buffer pools created in connection for these ports can be deleted */
  std::vector<core::tFrameworkElement::tHandle> deleted_rpc_ports;

  /*! Mutex for 'deleted_rpc_ports' access */
  rrlib::thread::tMutex deleted_rpc_ports_mutex;


  /*! Is provided element a shared port (to be announced to other peers)? */
  static bool IsSharedPort(core::tFrameworkElement& framework_element);

  virtual void OnEdgeChange(core::tRuntimeListener::tEvent change_type, core::tAbstractPort& source, core::tAbstractPort& target); // TODO: mark override with gcc 4.7
  virtual void OnFrameworkElementChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element); // TODO: mark override with gcc 4.7

  /*! Handles runtime changes from callbacks - and forwards info to 'incoming_structure_changes' queue */
  void ProcessRuntimeChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element, bool edge_change);

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
