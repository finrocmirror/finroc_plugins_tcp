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
/*!\file    plugins/tcp/internal/tPeerImplementation.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tPeerImplementation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tRuntimeEnvironment.h"
#include "plugins/network_transport/tNetworkConnections.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tNetworkPortInfo.h"
#include "plugins/tcp/internal/tRemotePart.h"
#include "plugins/tcp/internal/tServer.h"
#include "plugins/tcp/internal/util.h"

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

const int cLOW_PRIORITY_TASK_CALL_INTERVAL = 500; // milliseconds
const int cPROCESS_EVENTS_CALL_INTERVAL = 5; // milliseconds

template <bool REGULAR>
struct tProcessLowPriorityTasksCaller
{
  tPeerImplementation* implementation;

  tProcessLowPriorityTasksCaller(tPeerImplementation& implementation) : implementation(&implementation) {}

  void operator()(const boost::system::error_code& error)
  {
    if (!error)
    {
      implementation->ProcessLowPriorityTasks();
      if (REGULAR)
      {
        implementation->low_priority_tasks_timer.expires_from_now(boost::posix_time::milliseconds(cLOW_PRIORITY_TASK_CALL_INTERVAL));
        implementation->low_priority_tasks_timer.async_wait(*this);
      }
    }
  }
};

struct tProcessEventsCaller
{
  tPeerImplementation* implementation;

  tProcessEventsCaller(tPeerImplementation& implementation) : implementation(&implementation) {}

  void operator()(const boost::system::error_code& error)
  {
    if (!error)
    {
      implementation->ProcessEvents();
      implementation->event_processing_timer.expires_from_now(boost::posix_time::milliseconds(cPROCESS_EVENTS_CALL_INTERVAL));
      implementation->event_processing_timer.async_wait(*this);
    }
  }
};

struct tAddressConnectorTask
{
  tPeerImplementation* implementation;
  std::string connect_to;
  std::vector<boost::asio::ip::tcp::endpoint> endpoints;
  size_t current_endpoint_index;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket;

  tAddressConnectorTask(tPeerImplementation& implementation, const std::string &connect_to) :
    implementation(&implementation),
    connect_to(connect_to),
    endpoints(),
    current_endpoint_index(0),
    socket(new boost::asio::ip::tcp::socket(implementation.IOService()))
  {
    try
    {
      endpoints = ParseAndResolveNetworkAddress(connect_to);
      Connect("no endpoints resolved");
    }
    catch (const std::exception &e)
    {
      FINROC_LOG_PRINT(DEBUG, "Could not connect to ", connect_to, ". Reason: ", e.what());
      this->implementation->connect_to.push_back(connect_to);
    }
  }

  void Connect(const char* fail_reason)
  {
    if (current_endpoint_index < endpoints.size())
    {
      socket.reset(new boost::asio::ip::tcp::socket(implementation->IOService()));
      socket->async_connect(endpoints[current_endpoint_index], *this);
    }
    else
    {
      // put address back
      FINROC_LOG_PRINT(DEBUG, "Could not connect to ", connect_to, ". Reason: ", fail_reason);
      implementation->connect_to.push_back(connect_to);
    }
  }

  void operator()(const boost::system::error_code& error)
  {
    if (error)
    {
      current_endpoint_index++;
      Connect(error.message().c_str());
    }
    else
    {
      FINROC_LOG_PRINT(DEBUG, "Connected to ", connect_to, " (", endpoints[current_endpoint_index].address().to_string(), ")");
      tConnection::InitConnection(*implementation, socket, 0x7, NULL, true); // TODO: possibly use multiple connections
    }
  }
};

struct tConnectorTask
{
  tPeerImplementation* implementation;
  tPeerInfo* peer_info;
  size_t address_index;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket;
  std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator;

  tConnectorTask(tPeerImplementation& implementation, tPeerInfo& peer_info) :
    implementation(&implementation),
    peer_info(&peer_info),
    address_index(0),
    socket(),
    active_connect_indicator(new tPeerInfo::tActiveConnect(peer_info))
  {
    if (peer_info.addresses.size() == 0)
    {
      implementation.InferMissingAddresses();
    }
    if (peer_info.addresses.size() == 0)
    {
      FINROC_LOG_PRINT(WARNING, "Peer info with no addresses. This is likely a programming error.");
    }

    Connect();
  }

  void Connect()
  {
    if (address_index < peer_info->addresses.size())
    {
      boost::asio::ip::address address = peer_info->addresses[address_index];
      socket.reset(new boost::asio::ip::tcp::socket(implementation->IOService()));
      socket->async_connect(boost::asio::ip::tcp::endpoint(address, peer_info->uuid.port), *this);
    }
  }

  void operator()(const boost::system::error_code& error)
  {
    if (error)
    {
      // try next address
      address_index++;
      Connect();
    }
    else
    {
      tConnection::InitConnection(*implementation, socket, 0x7, active_connect_indicator); // TODO: possibly use multiple connections
    }
  }
};

/*! Server thread that does all the work for this server (and possibly peer) */
class tTCPThread : public rrlib::thread::tThread
{
public:

  tTCPThread(tPeerImplementation& implementation) :
    tThread("TCP Thread"),
    implementation(implementation)
  {
  }

private:
  virtual void Run()
  {
    try
    {
      implementation.low_priority_tasks_timer.async_wait(tProcessLowPriorityTasksCaller<true>(implementation));
      if (implementation.server)
      {
        implementation.server->Run();
      }
      else
      {
        implementation.io_service.run();
      }
    }
    catch (const std::exception& ex)
    {
      FINROC_LOG_PRINT(WARNING, "Thread exited with exception ", ex);
    }
  }

  virtual void StopThread()
  {
    implementation.io_service.stop();
  }

  tPeerImplementation& implementation;
};


tPeerImplementation::tPeerImplementation(core::tFrameworkElement& framework_element, const std::string& peer_name, tPeerType peer_type, const std::string& network_connection,
    int preferred_server_port, bool try_next_ports_if_occupied, bool auto_connect_to_all_peers, const std::string& server_listen_address) :
  framework_element(framework_element),
  network_connection(network_connection),
  connect_to(),
  this_peer(peer_type),
  other_peers(),
  //peer_list_revision(0),
  peer_list_changed(false),
  thread(),
  io_service(),
  low_priority_tasks_timer(io_service, boost::posix_time::milliseconds(cLOW_PRIORITY_TASK_CALL_INTERVAL)),
  event_processing_timer(io_service, boost::posix_time::milliseconds(5)),
  server(NULL),
  shared_ports(),
  shared_ports_mutex(),
  serve_structure(false),
  incoming_structure_changes(),
  actively_connect(false),
  pending_subscription_checks(),
  pending_subscription_checks_mutex(),
  pending_subscription_checks_copy(),
  event_loop_running(false),
  incoming_port_buffer_changes(),
  port_buffer_change_event_buffers(),
  deleted_rpc_ports(),
  deleted_rpc_ports_mutex()
{
  tSettings::GetInstance(); // initialize TCP settings
  this_peer.name = peer_name;

  // Retrieve host name
  char buffer[258];
  if (gethostname(buffer, 257))
  {
    this_peer.uuid.host_name = "No host name@" + std::to_string(rrlib::time::Now(true).time_since_epoch().count());
    FINROC_LOG_PRINT(ERROR, "Error retrieving host name.");
  }
  else if (std::string(buffer) == "localhost")
  {
    this_peer.uuid.host_name = "localhost@" + std::to_string(rrlib::time::Now(true).time_since_epoch().count());
    FINROC_LOG_PRINT(ERROR, "The hostname of this system is 'localhost' (according to the hostname() function). When using the finroc_tcp plugin, this is not allowed (a unique identifier for this Finroc runtime environment is derived from the hostname). Ideally, the hostname is the name under which the system can be found in the network using DNS lookup. Otherwise, please set it to a unique name in the network. For now, the current time is appended for the uuid: '", this_peer.uuid.host_name, "'.");
  }
  else
  {
    this_peer.uuid.host_name = buffer;
  }

  // Create server
  if (peer_type != tPeerType::CLIENT_ONLY)
  {
    server = new tServer(*this, preferred_server_port, try_next_ports_if_occupied, server_listen_address);
  }
}

tPeerImplementation::~tPeerImplementation()
{
  for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
  {
    if ((*it)->remote_part)
    {
      (*it)->remote_part->OnBoostAsioIoServiceDelete();
    }
  }

  io_service.stop();
  if (thread)
  {
    thread->StopThread();
    thread->Join();
  }

  core::tRuntimeEnvironment::GetInstance().RemoveListener(*this);
}

void tPeerImplementation::AddAddress(const boost::asio::ip::address& address)
{
  for (auto it = this_peer.addresses.begin(); it != this_peer.addresses.end(); ++it)
  {
    if (*it == address)
    {
      return;
    }
  }

  this_peer.addresses.push_back(address);
// peer_list_revision++;
  peer_list_changed = true;
}

void tPeerImplementation::AddPeerAddresses(tPeerInfo& existing_peer, const std::vector<boost::asio::ip::address>& addresses)
{
  for (auto & address : addresses)
  {
    bool found = false;
    for (auto & existing_address : existing_peer.addresses)
    {
      if (existing_address == address)
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      existing_peer.addresses.push_back(address);
      SetPeerListChanged();
    }
  }
}

void tPeerImplementation::Connect()
{
  actively_connect = true;

  if (network_connection.length() > 0)
  {
    connect_to.push_back(network_connection);
  }

  low_priority_tasks_timer.async_wait(tProcessLowPriorityTasksCaller<false>(*this)); // immediately trigger connecting
}

std::string tPeerImplementation::Connect(core::tAbstractPort& local_port, const std::string& remote_runtime_uuid,
    int remote_port_handle, const std::string remote_port_link, bool disconnect)
{
  for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
  {
    if ((*it)->remote_part && (*it)->uuid.ToString() == remote_runtime_uuid)
    {
      tRemotePart& part = *((*it)->remote_part);
      auto remote_port = part.remote_port_map.find(remote_port_handle);
      if (remote_port != part.remote_port_map.end())
      {
        if (!disconnect)
        {
          local_port.ConnectTo(remote_port->second->GetQualifiedLink(), core::tAbstractPort::tConnectDirection::AUTO, true);
          if (!local_port.IsConnectedTo(*remote_port->second))
          {
            return "Could not connect ports (see console output for reasons)";
          }
          return "";
        }
        else
        {
          local_port.DisconnectFrom(remote_port->second->GetQualifiedLink());
          if (local_port.IsConnectedTo(*remote_port->second))
          {
            return "Could not disconnect ports (see console output for reasons)";
          }
          return "";
        }
      }
      else
      {
        return "No remote port with handle " + std::to_string(remote_port_handle) + "found";
      }
    }
  }
  return "No remote runtime with UUID " + remote_runtime_uuid + " found";
}


void tPeerImplementation::DeserializePeerInfo(rrlib::serialization::tInputStream& stream, tPeerInfo& peer)
{
  stream >> peer.uuid;
  stream >> peer.peer_type;
  stream >> peer.name;
  int size = stream.ReadInt();
  peer.addresses.clear();
  for (int i = 0; i < size; ++i)
  {
    boost::asio::ip::address address;
    stream >> address;
    peer.addresses.push_back(address);
  }
}

tRemotePart* tPeerImplementation::GetRemotePart(const tUUID& uuid, tPeerType peer_type, const std::string& peer_name, const boost::asio::ip::address& address, bool never_forget)
{
  if (uuid == this->this_peer.uuid)
  {
    FINROC_LOG_PRINT(ERROR, "Remote part has the same UUID as this one: " + uuid.ToString());
    throw std::runtime_error("Remote part has the same UUID as this one: " + uuid.ToString());
  }

  for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
  {
    if ((*it)->uuid == uuid)
    {
      if (!(*it)->remote_part)
      {
        (*it)->remote_part = new tRemotePart(**it, framework_element, *this);
        (*it)->remote_part->Init();
      }
      (*it)->AddAddress(address);
#if 0
      if ((*it)->AddAddress(address))
      {
        peer_list_revision++;
      }
#endif
      (*it)->name = peer_name;
      (*it)->never_forget |= never_forget;
      return (*it)->remote_part;
    }
  }

  other_peers.emplace_back(new tPeerInfo(peer_type));
  tPeerInfo& info = **(other_peers.end() - 1);
  info.addresses.push_back(address);
  info.uuid = uuid;
  info.name = peer_name;
  info.never_forget = never_forget;
  info.remote_part = new tRemotePart(info, framework_element, *this);
  info.remote_part->Init();
  //peer_list_revision++;
  return info.remote_part;
}

void tPeerImplementation::InferMissingAddresses()
{
  for (auto & info : other_peers)
  {
    if (info->addresses.size() == 0)
    {
      for (auto & other_info : other_peers)
      {
        if (other_info != info && (other_info->uuid.host_name == info->uuid.host_name))
        {
          AddPeerAddresses(*info, other_info->addresses);
        }
      }
    }
  }
}

bool tPeerImplementation::IsSharedPort(core::tFrameworkElement& framework_element)
{
  return framework_element.IsPort() && framework_element.GetFlag(core::tFrameworkElement::tFlag::SHARED) &&
         (!framework_element.GetFlag(core::tFrameworkElement::tFlag::NETWORK_ELEMENT));
}

void tPeerImplementation::OnEdgeChange(core::tRuntimeListener::tEvent change_type, core::tAbstractPort& source, core::tAbstractPort& target)
{
  // Maintain network connection info for finstruct
  bool target_port_changed = false;
  UpdateNetworkConnectionInfo(change_type, source, target, target_port_changed);

  // Forward change to clients
  if (source.IsReady())
  {
    ProcessRuntimeChange(core::tRuntimeListener::tEvent::CHANGE, source, true);
  }
  if (target.IsReady() && target_port_changed)
  {
    ProcessRuntimeChange(core::tRuntimeListener::tEvent::CHANGE, target, true);
  }

  // Check subscriptions?
  if (source.IsReady())
  {
    tNetworkPortInfo* network_port_info = source.GetAnnotation<tNetworkPortInfo>();
    if (network_port_info)
    {
      network_port_info->CheckSubscription(pending_subscription_checks, pending_subscription_checks_mutex);
    }
  }
  if (target.IsReady())
  {
    tNetworkPortInfo* network_port_info = target.GetAnnotation<tNetworkPortInfo>();
    if (network_port_info)
    {
      network_port_info->CheckSubscription(pending_subscription_checks, pending_subscription_checks_mutex);
    }
  }
}

void tPeerImplementation::OnFrameworkElementChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element)
{
  // Maintain network connection info for finstruct
  if (change_type == core::tRuntimeListener::ADD && element.IsPort())
  {
    core::tAbstractPort& port = static_cast<core::tAbstractPort&>(element);
    for (auto it = port.OutgoingConnectionsBegin(); it != port.OutgoingConnectionsEnd(); ++it)
    {
      bool target_port_changed = false;
      UpdateNetworkConnectionInfo(change_type, port, *it, target_port_changed);
    }
  }

  ProcessRuntimeChange(change_type, element, false);

  // Check subscriptions?
  if (change_type == core::tRuntimeListener::tEvent::CHANGE)
  {
    tNetworkPortInfo* network_port_info = element.GetAnnotation<tNetworkPortInfo>();
    if (network_port_info)
    {
      network_port_info->CheckSubscription(pending_subscription_checks, pending_subscription_checks_mutex);
    }
  }
  if (change_type == core::tRuntimeListener::tEvent::ADD && element.IsPort())
  {
    // Check for any connected remote destination ports:
    // Network input port may already be initialized, while local connected output port is initialized now.
    // => Trigger subscription check in this case
    core::tAbstractPort& port = static_cast<core::tAbstractPort&>(element);
    for (auto it = port.OutgoingConnectionsBegin(); it != port.OutgoingConnectionsEnd(); ++it)
    {
      tNetworkPortInfo* network_port_info = it->GetAnnotation<tNetworkPortInfo>();
      if (network_port_info)
      {
        network_port_info->CheckSubscription(pending_subscription_checks, pending_subscription_checks_mutex);
      }
    }
  }


  // RPC port deletion?
  if (change_type == core::tRuntimeListener::tEvent::REMOVE && element.IsPort() &&
      rpc_ports::IsRPCType(static_cast<core::tAbstractPort&>(element).GetDataType()))
  {
    rrlib::thread::tLock lock(deleted_rpc_ports_mutex);
    deleted_rpc_ports.push_back(element.GetHandle());
  }
}

void tPeerImplementation::ProcessIncomingPeerInfo(const tPeerInfo& peer_info)
{
  tPeerInfo* existing_peer = NULL;
  if (peer_info.uuid == this_peer.uuid)
  {
    existing_peer = &this_peer;
  }

  for (auto & it : other_peers)
  {
    if (peer_info.uuid == it->uuid)
    {
      existing_peer = &(*it);
    }
  }

  if (existing_peer)
  {
    if (existing_peer->peer_type != peer_info.peer_type)
    {
      FINROC_LOG_PRINT(WARNING, "Peer type of existing peer has changed, will not update it.");
    }
    AddPeerAddresses(*existing_peer, peer_info.addresses);
  }
  else
  {
    other_peers.emplace_back(new tPeerInfo(peer_info.peer_type));
    tPeerInfo& info = **(other_peers.end() - 1);
    info.addresses = peer_info.addresses;
    info.uuid = peer_info.uuid;
    info.name = peer_info.name;
  }

}

void tPeerImplementation::ProcessEvents()
{
  rrlib::time::tTimestamp time_now = rrlib::time::Now();
  //FINROC_LOG_PRINT(DEBUG, "Called");

  // Process incoming structure changes
  ProcessRuntimeChangeEvents();

  // Process pending subscription checks
  {
    rrlib::thread::tLock lock(pending_subscription_checks_mutex);
    pending_subscription_checks_copy = pending_subscription_checks;
    pending_subscription_checks.clear();
  }
  for (auto it = pending_subscription_checks_copy.begin(); it != pending_subscription_checks_copy.end(); ++it)
  {
    core::tAbstractPort* port = core::tRuntimeEnvironment::GetInstance().GetPort(*it);
    if (port && port->IsReady())
    {
      tNetworkPortInfo* network_port_info = port->GetAnnotation<tNetworkPortInfo>();
      if (network_port_info)
      {
        network_port_info->DoSubscriptionCheck();
      }
    }
  }

  // Process incoming port buffer changes
  rrlib::concurrent_containers::tQueueFragment<tChangeEventPointer> port_changes = incoming_port_buffer_changes.DequeueAll();
  while (!port_changes.Empty())
  {
    tChangeEventPointer change_event = port_changes.PopFront();
    change_event->network_port_info->ProcessIncomingBuffer(change_event);
  }

  // Process active connections
  for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
  {
    if ((*it)->remote_part)
    {
      (*it)->remote_part->SendPendingMessages(time_now);
    }
  }
}

void tPeerImplementation::ProcessLowPriorityTasks()
{
  FINROC_LOG_PRINT(DEBUG_VERBOSE_2, "Alive ", rrlib::time::Now().time_since_epoch().count());

  // delete buffer pools created for RPC ports
  std::vector<core::tFrameworkElement::tHandle> deleted_ports;
  {
    rrlib::thread::tLock lock(deleted_rpc_ports_mutex);
    if (!deleted_rpc_ports.empty())
    {
      std::swap(deleted_ports, deleted_rpc_ports); // Move deleted ports to local variable and unlock
    }
  }
  if (!deleted_ports.empty())
  {
    for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
    {
      tPeerInfo& peer = **it;
      if (peer.remote_part)
      {
        peer.remote_part->RpcPortsDeleted(deleted_ports);
      }
    }
  }

  // connect to other peers
  if (actively_connect)
  {
    for (auto it = connect_to.begin(); it != connect_to.end(); ++it)
    {
      FINROC_LOG_PRINT(DEBUG, "Connecting to ", *it);
      tAddressConnectorTask connector_task(*this, *it);
    }
    connect_to.clear();

    for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
    {
      tPeerInfo& peer = **it;
      if ((!peer.connected) && (!peer.connecting) && (peer.peer_type != tPeerType::CLIENT_ONLY))
      {
        tConnectorTask connector_task(*this, peer);
      }
    }
  }

  // send peer list if needed
  if (peer_list_changed)
  {
    for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
    {
      tPeerInfo& peer = **it;
      tRemotePart *remote_part = peer.remote_part;
      if (remote_part && peer.connected)
      {
        std::shared_ptr<tConnection> management_connection = remote_part->GetManagementConnection();
        if (management_connection && management_connection->IsReady())
        {

          tPeerInfoMessage::Serialize(false, management_connection->CurrentWriteStream());

          SerializePeerInfo(management_connection->CurrentWriteStream(), this_peer);

          for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
          {
            SerializePeerInfo(management_connection->CurrentWriteStream(), **it);
          }

          management_connection->CurrentWriteStream().WriteBoolean(false);
          tPeerInfoMessage::FinishMessage(management_connection->CurrentWriteStream());

        }
      }
    }

    peer_list_changed = false;
  }
}

void tPeerImplementation::ProcessRuntimeChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element, bool edge_change)
{
  bool shared_port = IsSharedPort(element);
  bool serve_structure_copy = serve_structure.load();

  bool relevant_for_shared_port_client = shared_port && (!edge_change);
  bool relevant_for_structure_client = !element.GetFlag(core::tFrameworkElement::tFlag::NETWORK_ELEMENT) && (!edge_change);

  if ((relevant_for_shared_port_client || serve_structure_copy) && (change_type != core::tRuntimeListener::tEvent::PRE_INIT))
  {
    std::unique_ptr<tSerializedStructureChange> change(new tSerializedStructureChange(change_type, element, serve_structure_copy,
        relevant_for_shared_port_client ? common::tStructureExchange::SHARED_PORTS :
        (relevant_for_structure_client ? common::tStructureExchange::COMPLETE_STRUCTURE : common::tStructureExchange::FINSTRUCT)));

    if (relevant_for_shared_port_client)
    {
      rrlib::thread::tLock lock(shared_ports_mutex, false);
      if (change_type == core::tRuntimeListener::tEvent::ADD || change_type == core::tRuntimeListener::tEvent::CHANGE)
      {
        rrlib::serialization::tStackMemoryBuffer<2048> buffer;
        rrlib::serialization::tOutputStream stream(buffer);
        std::string string_buffer;
        common::tFrameworkElementInfo::Serialize(stream, element, common::tStructureExchange::SHARED_PORTS, string_buffer);
        stream.Flush();
        lock.Lock();
        shared_ports[element.GetHandle()] = CopyToNewFixedBuffer(buffer);
      }
      else if (change_type == core::tRuntimeListener::tEvent::REMOVE)
      {
        lock.Lock();
        shared_ports.erase(element.GetHandle());
      }
      incoming_structure_changes.Enqueue(std::move(change)); // do this with lock - to avoid inconsistencies
    }
    else
    {
      //FINROC_LOG_PRINT(DEBUG, "Enqueuing ", change.get(), " ", element.GetQualifiedName());
      incoming_structure_changes.Enqueue(std::move(change));
    }
  }
}

void tPeerImplementation::ProcessRuntimeChangeEvents()
{
  rrlib::concurrent_containers::tQueueFragment<std::unique_ptr<tSerializedStructureChange>> incoming_structure_changes_fragment = incoming_structure_changes.DequeueAll();
  while (!incoming_structure_changes_fragment.Empty())
  {
    std::unique_ptr<tSerializedStructureChange> incoming_structure_change = incoming_structure_changes_fragment.PopFront();
    //FINROC_LOG_PRINT(DEBUG, "Dequeuing ", incoming_structure_change.get());
    for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
    {
      if ((*it)->remote_part)
      {
        if (static_cast<size_t>((*it)->remote_part->GetDesiredStructureInfo()) >= static_cast<size_t>(incoming_structure_change->MinimumRelevantLevel()))
        {
          (*it)->remote_part->SendStructureChange(*incoming_structure_change);
        }
      }
    }
  }
}

void tPeerImplementation::RunEventLoop()
{
  if (!event_loop_running)
  {
    event_loop_running = true;
    event_processing_timer.expires_from_now(boost::posix_time::milliseconds(cPROCESS_EVENTS_CALL_INTERVAL));
    event_processing_timer.async_wait(tProcessEventsCaller(*this));
  }
}

//FIXME: remove in next Finroc version
bool IsLoopbackAddress(const boost::asio::ip::address& address)
{
  return address.is_v4() ? (address.to_v4() == boost::asio::ip::address_v4::loopback()) : (address.to_v6() == boost::asio::ip::address_v6::loopback());
}

void tPeerImplementation::SerializePeerInfo(rrlib::serialization::tOutputStream& stream, const tPeerInfo& peer)
{
  if ((&peer == &this_peer || peer.connected) && peer.peer_type != tPeerType::CLIENT_ONLY)
  {
    stream << true;
    stream << peer.uuid;
    stream << peer.peer_type;
    stream << peer.name;

    // count non-loopback addresses
    int address_count = 0;
    for (auto & it : peer.addresses)
    {
      // if (!it.is_loopback()) FIXME: replace in next Finroc version
      if (!IsLoopbackAddress(it))
      {
        address_count++;
      }
    }
    stream.WriteInt(address_count);

    // serialize non-loopback addresses
    for (auto & it : peer.addresses)
    {
      // if (!it.is_loopback()) FIXME: replace in next Finroc version
      if (!IsLoopbackAddress(it))
      {
        stream << it;
      }
    }
  }
}

rrlib::serialization::tMemoryBuffer tPeerImplementation::SerializeSharedPorts(common::tRemoteTypes& connection_type_encoder)
{
  rrlib::thread::tLock lock(shared_ports_mutex);
  ProcessRuntimeChangeEvents();  // to make sure we don't get any shared port events twice
  rrlib::serialization::tMemoryBuffer buffer(shared_ports.size() * 200);
  rrlib::serialization::tOutputStream stream(buffer, connection_type_encoder);
  stream.WriteInt(0); // Placeholder for size
  stream << rrlib::rtti::tDataType<std::string>(); // write a data type for initialization
  for (auto it = shared_ports.begin(); it != shared_ports.end(); ++it)
  {
    stream << it->first;
    stream.Write(it->second);
  }
  stream.WriteInt(0); // size of next packet
  stream.Close();
  buffer.GetBuffer().PutInt(0, buffer.GetSize() - 8);
  return buffer;
}

void tPeerImplementation::StartServer()
{
  core::tRuntimeEnvironment::GetInstance().AddListener(*this);

  // Collect existing shared ports and store serialized information about them
  rrlib::serialization::tStackMemoryBuffer<2048> buffer;
  rrlib::serialization::tOutputStream stream(buffer);
  std::string string_buffer;
  enum { cPORT_BUFFER_SIZE = 2048 };
  core::tAbstractPort* port_buffer[cPORT_BUFFER_SIZE];
  typename core::tFrameworkElement::tHandle start_handle = 0;
  while (true)
  {
    size_t port_count = core::tRuntimeEnvironment::GetInstance().GetAllPorts(port_buffer, cPORT_BUFFER_SIZE, start_handle);
    for (size_t i = 0; i < port_count; i++)
    {
      core::tAbstractPort& port = *port_buffer[i];
      if (IsSharedPort(port))
      {
        stream.Reset();
        common::tFrameworkElementInfo::Serialize(stream, port, common::tStructureExchange::SHARED_PORTS, string_buffer);
        stream.Flush();
        shared_ports.insert(std::pair<core::tFrameworkElement::tHandle, rrlib::serialization::tFixedBuffer>(port.GetHandle(), CopyToNewFixedBuffer(buffer)));
      }
    }
    if (port_count < cPORT_BUFFER_SIZE)
    {
      break;
    }
    start_handle = port_buffer[cPORT_BUFFER_SIZE - 1]->GetHandle() + 1;
  };

  // Start TCP Thread
  StartThread();
}

void tPeerImplementation::StartThread()
{
  assert(!thread);
  thread = (new tTCPThread(*this))->GetSharedPtr();
  thread->SetAutoDelete();
  thread->Start();
}

void tPeerImplementation::UpdateNetworkConnectionInfo(core::tRuntimeListener::tEvent change_type, core::tAbstractPort& source, core::tAbstractPort& target, bool& target_port_changed)
{
  tNetworkPortInfo* target_port_info = target.GetAnnotation<tNetworkPortInfo>();
  bool destination_is_source = false;
  core::tAbstractPort* connection_annotated = &source;
  if ((!target_port_info) || target_port_info->IsServerPort())
  {
    target_port_info = source.GetAnnotation<tNetworkPortInfo>();
    destination_is_source = true;
    connection_annotated = &target;
  }

  if (target_port_info && (!target_port_info->IsServerPort()))
  {
    target_port_changed = destination_is_source;
    network_transport::tNetworkConnections* connections_annotation = connection_annotated->GetAnnotation<network_transport::tNetworkConnections>();
    if (change_type == core::tRuntimeListener::tEvent::ADD)
    {
      if (!connections_annotation)
      {
        connections_annotation = &connection_annotated->EmplaceAnnotation<network_transport::tNetworkConnections>();
      }
      connections_annotation->Add(network_transport::tNetworkConnection(target_port_info->GetRemotePart().peer_info.uuid.ToString(), target_port_info->GetRemoteHandle(), destination_is_source));
    }
    else if (change_type == core::tRuntimeListener::tEvent::REMOVE && connections_annotation)
    {
      connections_annotation->Remove(network_transport::tNetworkConnection(target_port_info->GetRemotePart().peer_info.uuid.ToString(), target_port_info->GetRemoteHandle(), destination_is_source));
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
