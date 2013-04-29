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
  boost::asio::ip::tcp::endpoint connect_to;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket;

  tAddressConnectorTask(tPeerImplementation& implementation, boost::asio::ip::tcp::endpoint connect_to) :
    implementation(&implementation),
    connect_to(connect_to),
    socket(new boost::asio::ip::tcp::socket(implementation.IOService()))
  {
    socket->async_connect(connect_to, *this);
  }

  void operator()(const boost::system::error_code& error)
  {
    if (error)
    {
      // put address back
      FINROC_LOG_PRINT(DEBUG, "Could not connect to ", connect_to.address().to_string(), " port ", connect_to.port(), ". Reason: ", error.message());
      implementation->connect_to.push_back(connect_to);
    }
    else
    {
      FINROC_LOG_PRINT(DEBUG, "Connected to ", connect_to.address().to_string());
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
  peer_list_revision(0),
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
    this_peer.uuid.host_name = "No host name";
    FINROC_LOG_PRINT(ERROR, "Error retrieving host name.");
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
  peer_list_revision++;
}

void tPeerImplementation::Connect()
{
  actively_connect = true;

  if (network_connection.length() > 0)
  {
    try
    {
      connect_to.push_back(ParseAndResolveNetworkAddress(network_connection));
    }
    catch (const std::exception& ex)
    {
      FINROC_LOG_PRINT(WARNING, ex.what());
    }
  }

  low_priority_tasks_timer.async_wait(tProcessLowPriorityTasksCaller<false>(*this)); // immediately trigger connecting
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
  peer_list_revision++;
  return info.remote_part;
}

bool tPeerImplementation::IsSharedPort(core::tFrameworkElement& framework_element)
{
  return framework_element.IsPort() && framework_element.GetFlag(core::tFrameworkElement::tFlag::SHARED) &&
         (!framework_element.GetFlag(core::tFrameworkElement::tFlag::NETWORK_ELEMENT));
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
      FINROC_LOG_PRINT(DEBUG, "Connecting to ", it->address().to_string());
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
}

void tPeerImplementation::ProcessRuntimeChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element, bool edge_change)
{
  bool shared_port = IsSharedPort(element);
  bool serve_structure_copy = serve_structure.load();

  bool relevant_for_shared_port_client = shared_port && (!edge_change);
  bool relevant_for_structure_client = !framework_element.GetFlag(core::tFrameworkElement::tFlag::NETWORK_ELEMENT) && (!edge_change);

  if ((relevant_for_shared_port_client || serve_structure_copy) && (change_type != core::tRuntimeListener::tEvent::PRE_INIT))
  {
    std::unique_ptr<tSerializedStructureChange> change(new tSerializedStructureChange(change_type, element, serve_structure_copy,
        relevant_for_shared_port_client ? common::tStructureExchange::SHARED_PORTS :
        (relevant_for_structure_client ? common::tStructureExchange::COMPLETE_STRUCTURE : common::tStructureExchange::FINSTRUCT)));

    if (relevant_for_shared_port_client)
    {
      rrlib::thread::tLock lock(shared_ports_mutex);
      if (change_type == core::tRuntimeListener::tEvent::ADD || change_type == core::tRuntimeListener::tEvent::CHANGE)
      {
        rrlib::serialization::tStackMemoryBuffer<2048> buffer;
        rrlib::serialization::tOutputStream stream(buffer);
        std::string string_buffer;
        common::tFrameworkElementInfo::Serialize(stream, element, common::tStructureExchange::SHARED_PORTS, string_buffer);
        stream.Flush();
        shared_ports.insert(std::pair<core::tFrameworkElement::tHandle, rrlib::serialization::tFixedBuffer>(element.GetHandle(), CopyToNewFixedBuffer(buffer)));
      }
      else if (change_type == core::tRuntimeListener::tEvent::REMOVE)
      {
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

void tPeerImplementation::RuntimeChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element)
{
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

  // RPC port deletion?
  if (change_type == core::tRuntimeListener::tEvent::REMOVE && element.IsPort() &&
      rpc_ports::IsRPCType(static_cast<core::tAbstractPort&>(element).GetDataType()))
  {
    rrlib::thread::tLock lock(deleted_rpc_ports_mutex);
    deleted_rpc_ports.push_back(element.GetHandle());
  }
}

void tPeerImplementation::RuntimeEdgeChange(core::tRuntimeListener::tEvent change_type, core::tAbstractPort& source, core::tAbstractPort& target)
{
  if (source.IsReady())
  {
    ProcessRuntimeChange(core::tRuntimeListener::tEvent::CHANGE, source, true);
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
  buffer.GetBuffer()->PutInt(0, buffer.GetSize() - 8);
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



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
