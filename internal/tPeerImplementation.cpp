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

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tConnection.h"
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
typedef network_transport::runtime_info::tStructureExchange tStructureExchange;
typedef network_transport::generic_protocol::tRemoteRuntime tRemoteRuntime;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

namespace
{

class tRemoteRuntimeRemover : public core::tAnnotation
{
public:
  tRemoteRuntimeRemover(std::shared_ptr<tPeerInfo>& peer_pointer) :
    peer_pointer(peer_pointer)
  {}

  virtual void OnManagedDelete() override
  {
    if (peer_pointer->remote_runtime)
    {
      FINROC_LOG_PRINT(DEBUG, "Disconnected from ", peer_pointer->remote_runtime->GetName());
      peer_pointer->remote_runtime = nullptr;
    }
  }

  std::shared_ptr<tPeerInfo> peer_pointer;
};

boost::posix_time::milliseconds ToBoostPosixTime(const rrlib::time::tDuration& d)
{
  return boost::posix_time::milliseconds(std::chrono::duration_cast<std::chrono::milliseconds>(d).count());
}

}

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
        implementation->low_priority_tasks_timer.expires_from_now(ToBoostPosixTime(implementation->par_process_low_priority_tasks_call_interval.Get()));
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
      implementation->event_processing_timer.expires_from_now(ToBoostPosixTime(implementation->par_process_events_call_interval.Get()));
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
      tConnection::TryToEstablishConnection(*implementation, socket, 0x7, NULL, true); // TODO: possibly use multiple connections
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
      tConnection::TryToEstablishConnection(*implementation, socket, 0x7, active_connect_indicator); // TODO: possibly use multiple connections
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
  virtual void Run() override
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
        implementation.io_service->run();
      }
    }
    catch (const std::exception& ex)
    {
      FINROC_LOG_PRINT(WARNING, "TCP thread exited with exception ", ex);
    }
  }

  virtual void StopThread() override
  {
    implementation.io_service->stop();
  }

  tPeerImplementation& implementation;
};

//FIXME: remove in next Finroc version
bool IsLoopbackAddress(const boost::asio::ip::address& address)
{
  return address.is_v4() ? (address.to_v4() == boost::asio::ip::address_v4::loopback()) : (address.to_v6() == boost::asio::ip::address_v6::loopback());
}


tPeerImplementation::tPeerImplementation() :
  this_peer(tPeerType::UNSPECIFIED),
  other_peers(),
  //peer_list_revision(0),
  peer_list_changed(false),
  thread(),
  io_service(new boost::asio::io_service()),
  low_priority_tasks_timer(*io_service, boost::posix_time::milliseconds(500)),  // this is only the initial wait
  event_processing_timer(*io_service, boost::posix_time::milliseconds(5)),
  server(nullptr),
  actively_connect(false),
  event_loop_running(false)
{
}

tPeerImplementation::~tPeerImplementation()
{
  io_service->stop();
  if (thread)
  {
    thread->StopThread();
    thread->Join();
  }
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
  low_priority_tasks_timer.async_wait(tProcessLowPriorityTasksCaller<false>(*this)); // immediately trigger connecting
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

network_transport::generic_protocol::tRemoteRuntime* tPeerImplementation::GetRemoteRuntime(std::shared_ptr<tConnection>& connection, const tUUID& uuid, tPeerType peer_type, const std::string& peer_name, const boost::asio::ip::address& address, bool never_forget)
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
      if (!(*it)->remote_runtime)
      {
        (*it)->remote_runtime = new tRemoteRuntime(*this, connection, *GetPluginRootFrameworkElement(), uuid.ToString());
        (*it)->remote_runtime->EmplaceAnnotation<tRemoteRuntimeRemover>(*it);
        (*it)->remote_runtime->Init();
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
      return (*it)->remote_runtime;
    }
  }

  other_peers.emplace_back(new tPeerInfo(peer_type));
  tPeerInfo& info = *other_peers.back();
  info.addresses.push_back(address);
  info.uuid = uuid;
  info.name = peer_name;
  info.never_forget = never_forget;
  info.remote_runtime = new tRemoteRuntime(*this, connection, *GetPluginRootFrameworkElement(), uuid.ToString());
  info.remote_runtime->EmplaceAnnotation<tRemoteRuntimeRemover>(other_peers.back());
  info.remote_runtime->Init();
  //peer_list_revision++;
  return info.remote_runtime;
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

void tPeerImplementation::Init(rrlib::xml::tNode* config_node)
{
  tNetworkTransportPlugin::Init(config_node);
  this_peer.peer_type = par_peer_type.Get();
  this_peer.name = network_transport::generic_protocol::tLocalRuntimeInfo::GetName();

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
  if (this_peer.peer_type != tPeerType::CLIENT_ONLY)
  {
    server = new tServer(*this);
  }
}

void tPeerImplementation::OnStartServingStructure()
{
  Connect();
  StartServer();
}

void tPeerImplementation::ProcessIncomingPeerInfo(const tPeerInfo& peer_info)
{
  tPeerInfo* existing_peer = nullptr;
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
  ProcessLocalRuntimeCallsToSend();
  ProcessLocalRuntimePortDataChanges();
  ProcessLocalRuntimeStructureChanges();

  // Process active connections
  for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
  {
    if ((*it)->remote_runtime)
    {
      (*it)->remote_runtime->SendPendingMessages(time_now);
    }
  }
}

void tPeerImplementation::ProcessLowPriorityTasks()
{
  FINROC_LOG_PRINT(DEBUG_VERBOSE_2, "Alive ", rrlib::time::Now().time_since_epoch().count());

  // connect to other peers
  if (actively_connect)
  {
    if (par_connect_to.HasChanged())
    {
      auto current_connect_to = par_connect_to.GetPointer();
      for (const std::string & address : (*current_connect_to))
      {
        if (connected_to.count(address) == 0)
        {
          connect_to.push_back(address);
          connected_to.insert(address);
        }
      }
      par_connect_to.ResetChanged();
    }

    for (auto it = connect_to.begin(); it != connect_to.end(); ++it)
    {
      FINROC_LOG_PRINT(DEBUG, "Connecting to ", *it);
      tAddressConnectorTask connector_task(*this, *it);
    }
    connect_to.clear();

    for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
    {
      tPeerInfo& peer = **it;
      if ((!peer.remote_runtime) && (!peer.connecting) && (peer.peer_type != tPeerType::CLIENT_ONLY) &&
          (par_auto_connect_to_all_peers.Get() || peer.never_forget))
      {
        tConnectorTask connector_task(*this, peer);
      }
    }
  }

  // send peer list if needed
  if (peer_list_changed)
  {
    for (auto & remote_runtime : ConnectedRuntimes())
    {
      tConnection& connection = static_cast<tConnection&>(*remote_runtime->GetPrimaryConnection());
      if (connection.IsReady())
      {
        auto& stream = connection.CurrentWriteStream();
        network_transport::generic_protocol::tPeerInfoMessage::Serialize(false, true, stream);
        SerializePeerInfo(stream, this_peer);
        for (auto it = other_peers.begin(); it != other_peers.end(); ++it)
        {
          SerializePeerInfo(stream, **it);
        }
        stream.WriteBoolean(false);
        network_transport::generic_protocol::tPeerInfoMessage::FinishMessage(stream);
      }
    }

    peer_list_changed = false;
  }
}

void tPeerImplementation::RunEventLoop()
{
  if (!event_loop_running)
  {
    event_loop_running = true;
    event_processing_timer.expires_from_now(ToBoostPosixTime(par_process_events_call_interval.Get()));
    event_processing_timer.async_wait(tProcessEventsCaller(*this));
  }
}

void tPeerImplementation::SerializePeerInfo(rrlib::serialization::tOutputStream& stream, const tPeerInfo& peer)
{
  if ((&peer == &this_peer || peer.remote_runtime) && peer.peer_type != tPeerType::CLIENT_ONLY)
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

void tPeerImplementation::StartServer()
{
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
