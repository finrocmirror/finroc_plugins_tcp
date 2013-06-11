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
/*!\file    plugins/tcp/internal/tRemotePart.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tRemotePart.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tRuntimeEnvironment.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tNetworkPortInfo.h"
#include "plugins/tcp/internal/tNetworkServerPort.h"
#include "plugins/tcp/internal/tPeerImplementation.h"

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

/*!
 * Deserialization scope for RPC calls.
 * Buffer pool is created/provided when needed
 */
class tRPCDeserializationScope : public data_ports::api::tDeserializationScope
{
public:
  tRPCDeserializationScope(core::tFrameworkElement::tHandle local_port_handle,
                           std::map<core::tFrameworkElement::tHandle, std::unique_ptr<data_ports::standard::tMultiTypePortBufferPool>>& rpc_call_buffer_pools) :
    tDeserializationScope(),
    local_port_handle(local_port_handle),
    rpc_call_buffer_pools(rpc_call_buffer_pools)
  {}

private:
  virtual data_ports::standard::tMultiTypePortBufferPool& ObtainBufferPool()
  {
    auto it = rpc_call_buffer_pools.find(local_port_handle);
    if (it == rpc_call_buffer_pools.end())
    {
      it = rpc_call_buffer_pools.insert(std::pair<core::tFrameworkElement::tHandle, std::unique_ptr<data_ports::standard::tMultiTypePortBufferPool>>(
                                          local_port_handle, std::unique_ptr<data_ports::standard::tMultiTypePortBufferPool>(new data_ports::standard::tMultiTypePortBufferPool()))).first;
    }
    return *(it->second);
  }

  core::tFrameworkElement::tHandle local_port_handle;
  std::map<core::tFrameworkElement::tHandle, std::unique_ptr<data_ports::standard::tMultiTypePortBufferPool>>& rpc_call_buffer_pools;
};


tRemotePart::tRemotePart(tPeerInfo& peer_info, core::tFrameworkElement& parent, tPeerImplementation& peer_implementation) :
  tFrameworkElement(&parent, peer_info.ToString(), tFlag::NETWORK_ELEMENT),
  peer_info(peer_info),
  peer_implementation(peer_implementation),
  express_connection(),
  bulk_connection(),
  management_connection(),
  send_structure_info(common::tStructureExchange::NONE),
  global_links(NULL /*new core::tFrameworkElement(this, "global", tFlag::NETWORK_ELEMENT | tFlag::GLOBALLY_UNIQUE_LINK | tFlag::ALTERNATIVE_LINK_ROOT)*/),
  server_ports(NULL),
  server_port_map(),
  remote_port_map(),
  ports_with_express_data_to_send(),
  ports_with_bulk_data_to_send(),
  not_ready_calls(),
  calls_awaiting_response(),
  next_call_id(0),
  pull_calls_awaiting_response()
{}

tRemotePart::~tRemotePart()
{}

bool tRemotePart::AddConnection(std::shared_ptr<tConnection> connection)
{
  bool added = false;
  if (connection->flags & static_cast<int>(tConnectionFlag::MANAGEMENT_DATA))
  {
    if (management_connection)
    {
      return false;
    }
    management_connection = connection;
    added = true;
  }
  if (connection->flags & static_cast<int>(tConnectionFlag::EXPRESS_DATA))
  {
    if (express_connection)
    {
      return false;
    }
    express_connection = connection;
    added = true;
  }
  if (connection->flags & static_cast<int>(tConnectionFlag::BULK_DATA))
  {
    if (bulk_connection)
    {
      return false;
    }
    bulk_connection = connection;
    added = true;
  }
  if (management_connection && express_connection && bulk_connection)
  {
    peer_info.connected = true;
    FINROC_LOG_PRINT(DEBUG, "Connected to " + peer_info.ToString());
    connection->peer.RunEventLoop();
    // TODO: possibly increase peer list revision
  }
  return added;
}

void tRemotePart::AddRemotePort(common::tFrameworkElementInfo& info)
{
  if (info.link_count == 0)
  {
    FINROC_LOG_PRINT(WARNING, "Remote shared port has no links. Ignoring.");
    return;
  }
  if (info.type == NULL)
  {
    FINROC_LOG_PRINT(WARNING, "Remote shared port '", info.links[0].name, "' has unknown type. Ignoring.");
    return;
  }
  if (remote_port_map.find(info.handle) != remote_port_map.end())
  {
    FINROC_LOG_PRINT(WARNING, "Received info on remote shared port '", info.links[0].name, "' twice.");
    return;
  }

  core::tPortWrapperBase created_port;
  if (data_ports::IsDataFlowType(info.type))
  {
    constexpr tFlags cKEEP_FLAGS = tFlag::ACCEPTS_DATA | tFlag::EMITS_DATA | tFlag::OUTPUT_PORT | tFlag::EXPRESS_PORT | tFlag::TOOL_PORT | tFlag::PUSH_STRATEGY_REVERSE;
    info.changeable_info.flags = tFlags(info.changeable_info.flags.Raw() & cKEEP_FLAGS.Raw()) | tFlag::NETWORK_ELEMENT | tFlag::VOLATILE;
    data_ports::tGenericPort port(info.links[0].name, info.links[0].unique ? GetGlobalLinksElement() : this, info.type, info.changeable_info.flags);
    port.GetWrapped()->SetMinNetUpdateIntervalRaw(info.changeable_info.min_net_update_time);
    tNetworkPortInfo* network_port_info = new tNetworkPortInfo(*this, info.handle, info.changeable_info.strategy, false, *port.GetWrapped());
    if (info.changeable_info.strategy > 0)
    {
      port.GetWrapped()->SetPushStrategy(true);
    }
    created_port = port;
    port.AddPortListenerForPointer(*network_port_info);
    port.SetPullRequestHandler(this);
  }
  else if (rpc_ports::IsRPCType(info.type) && (!info.changeable_info.flags.Get(core::tFrameworkElementFlag::OUTPUT_PORT)))
  {
    constexpr tFlags cKEEP_FLAGS = tFlag::EXPRESS_PORT | tFlag::TOOL_PORT;
    info.changeable_info.flags = tFlags(info.changeable_info.flags.Raw() & cKEEP_FLAGS.Raw()) | tFlag::NETWORK_ELEMENT | tFlag::VOLATILE;
    tNetworkServerPort* port = new tNetworkServerPort(*this, info.links[0].unique ? GetGlobalLinksElement() : this, info.links[0].name, info.type, info.changeable_info.flags);
    new tNetworkPortInfo(*this, info.handle, info.changeable_info.strategy, false, *port);
    created_port = core::tPortWrapperBase(port);
  }

  if (created_port.GetWrapped())
  {
    for (size_t i = 1; i < info.link_count; i++)
    {
      created_port.GetWrapped()->Link(info.links[i].unique ? *GetGlobalLinksElement() : *this, info.links[i].name);
    }
    remote_port_map.insert(std::pair<tFrameworkElementHandle, core::tAbstractPort*>(info.handle, created_port.GetWrapped()));
    created_port.Init();
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Created remote port ", created_port.GetWrapped()->GetQualifiedName(), " ", info.handle);
  }
}

core::tFrameworkElement* tRemotePart::GetGlobalLinksElement()
{
  if (!global_links)
  {
    global_links = new core::tFrameworkElement(this, "global", tFlag::NETWORK_ELEMENT | tFlag::GLOBALLY_UNIQUE_LINK | tFlag::ALTERNATIVE_LINK_ROOT);
    global_links->Init();
  }
  return global_links;
}

core::tFrameworkElement* tRemotePart::GetServerPortsElement()
{
  if (!server_ports)
  {
    server_ports = new core::tFrameworkElement(this, "server ports");
    server_ports->Init();
  }
  return server_ports;
}

data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject> tRemotePart::OnPullRequest(data_ports::tGenericPort& origin)
{
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Pull request received");
  tPullCallInfo pull_call_info(*this, *origin.GetWrapped());
  rpc_ports::tFuture<data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject>> pull_future(pull_call_info.promise->GetFuture());
  GetPeerImplementation().IOService().post(pull_call_info);
  try
  {
    return pull_future.Get(data_ports::cPULL_TIMEOUT);
  }
  catch (const std::exception& exception)
  {
  }
  FINROC_LOG_PRINT(DEBUG_WARNING, "Pull call timed out (", origin.GetWrapped()->GetQualifiedName(), ")");
  return data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject>();
}

void tRemotePart::PortDeleted(tNetworkPortInfo* deleted_port)
{
  ports_with_express_data_to_send.erase(std::remove(ports_with_express_data_to_send.begin(), ports_with_express_data_to_send.end(),
                                        deleted_port), ports_with_express_data_to_send.end());
  ports_with_bulk_data_to_send.erase(std::remove(ports_with_bulk_data_to_send.begin(), ports_with_bulk_data_to_send.end(),
                                     deleted_port), ports_with_bulk_data_to_send.end());
  if (deleted_port->IsServerPort())
  {
    size_t erased = server_port_map.erase(deleted_port->GetServedPortHandle());
    if (!erased)
    {
      FINROC_LOG_PRINT(ERROR, "Deleted server port was not im map (This is a programming error)");
    }
  }
}

bool tRemotePart::ProcessMessage(tOpCode opcode, rrlib::serialization::tMemoryBuffer& buffer, common::tRemoteTypes& remote_types, tConnection& connection)
{
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Processing message ", make_builder::GetEnumString(opcode));
  rrlib::serialization::tInputStream stream(buffer, remote_types);

  if (opcode == tOpCode::PORT_VALUE_CHANGE || opcode == tOpCode::SMALL_PORT_VALUE_CHANGE || opcode == tOpCode::SMALL_PORT_VALUE_CHANGE_WITHOUT_TIMESTAMP)
  {
    tPortValueChange message;
    message.Deserialize(stream, false);

    core::tAbstractPort* port = core::tRuntimeEnvironment::GetInstance().GetPort(message.Get<0>());
    if (port && port->IsReady() && data_ports::IsDataFlowType(port->GetDataType()))
    {
      if (!port->GetAnnotation<tNetworkPortInfo>())
      {
        auto it = server_port_map.find(message.Get<0>());
        if (it != server_port_map.end())
        {
          port = it->second.GetWrapped();
        }
        else
        {
          return false; // received value for neither remote proxy port nor server port for local port
        }
      }

      data_ports::tGenericPort generic_port = data_ports::tGenericPort::Wrap(*port);
      bool another_value = false;
      do
      {
        rrlib::time::tTimestamp timestamp = rrlib::time::cNO_TIME;
        data_ports::tChangeStatus change_type;
        stream >> change_type;
        if (opcode != tOpCode::SMALL_PORT_VALUE_CHANGE_WITHOUT_TIMESTAMP)
        {
          stream >> timestamp;
        }
        data_ports::tPortDataPointer<rrlib::rtti::tGenericObject> buffer = generic_port.GetUnusedBuffer();
        buffer.SetTimestamp(timestamp);
        buffer->Deserialize(stream, message.Get<1>());
        generic_port.BrowserPublish(buffer, false, change_type);
        another_value = stream.ReadBoolean();
      }
      while (another_value);

      message.FinishDeserialize(stream);
    }
  }
  else if (opcode == tOpCode::RPC_CALL)
  {
    tRPCCall message;
    message.Deserialize(stream, false);
    rpc_ports::tCallType call_type = message.Get<1>();

    rrlib::rtti::tType rpc_interface_type;
    uint8_t function_index;
    stream >> rpc_interface_type >> function_index;

    rpc_ports::internal::tRPCInterfaceTypeInfo* type_info = rpc_interface_type.GetAnnotation<rpc_ports::internal::tRPCInterfaceTypeInfo>();
    if ((!type_info) || (!rpc_ports::IsRPCType(rpc_interface_type)))
    {
      FINROC_LOG_PRINT(ERROR, "Type ", rpc_interface_type.GetName(), " is no RPC type. Ignoring call.");
      return false;
    }
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Received ", make_builder::GetEnumString(call_type));

    if (call_type == rpc_ports::tCallType::RPC_MESSAGE || call_type == rpc_ports::tCallType::RPC_REQUEST)
    {
      core::tAbstractPort* port = core::tRuntimeEnvironment::GetInstance().GetPort(message.Get<0>());
      if (port && rpc_interface_type == port->GetDataType())
      {
        tRPCDeserializationScope deserialization_scope(message.Get<0>(), connection.rpc_call_buffer_pools);
        if (call_type == rpc_ports::tCallType::RPC_MESSAGE)
        {
          type_info->DeserializeMessage(stream, static_cast<rpc_ports::internal::tRPCPort&>(*port), function_index);
        }
        else
        {
          type_info->DeserializeRequest(stream, static_cast<rpc_ports::internal::tRPCPort&>(*port), function_index, *this);
        }
      }
    }
    else // type is RPC response
    {
      tCallId call_id;
      stream >> call_id;

      tCallPointer call_awaiting_this_response;
      for (auto it = calls_awaiting_response.begin(); it != calls_awaiting_response.end(); ++it)
      {
        if (it->second->GetCallId() == call_id)
        {
          call_awaiting_this_response = std::move(it->second);
          calls_awaiting_response.erase(it);
          break;
        }
      }
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Call awaiting: ", call_awaiting_this_response.get());
      if (call_awaiting_this_response)
      {
        core::tAbstractPort* port = core::tRuntimeEnvironment::GetInstance().GetPort(call_awaiting_this_response->GetLocalPortHandle());
        if (port)
        {
          tRPCDeserializationScope deserialization_scope(call_awaiting_this_response->GetLocalPortHandle(), connection.rpc_call_buffer_pools);
          type_info->DeserializeResponse(stream, function_index, *this, call_awaiting_this_response.get());
          return false;
        }
      }
      call_awaiting_this_response.reset();
      type_info->DeserializeResponse(stream, function_index, *this, call_awaiting_this_response.get());
    }
  }
  else if (opcode == tOpCode::PULLCALL)
  {
    tPullCall message;
    message.Deserialize(stream);

    core::tAbstractPort* port = core::tRuntimeEnvironment::GetInstance().GetPort(message.Get<0>());
    rrlib::serialization::tOutputStream& write_stream = express_connection->CurrentWriteStream();
    if (port && port->IsReady() && data_ports::IsDataFlowType(port->GetDataType()))
    {
      tPullCallReturn::Serialize(false, write_stream, message.Get<1>(), false);
      data_ports::tGenericPort data_port = data_ports::tGenericPort::Wrap(*port);
      data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject> pulled_buffer =
        data_port.GetPointer(data_ports::tStrategy::PULL_IGNORING_HANDLER_ON_THIS_PORT);
      write_stream << pulled_buffer->GetType() << pulled_buffer.GetTimestamp();
      pulled_buffer->Serialize(write_stream, message.Get<2>());
      tPullCallReturn::FinishMessage(write_stream);
    }
    else
    {
      tPullCallReturn::Serialize(true, express_connection->CurrentWriteStream(), message.Get<1>(), true);
    }
  }
  else if (opcode == tOpCode::PULLCALL_RETURN)
  {
    tPullCallReturn message;
    message.Deserialize(stream, false);

    for (auto it = pull_calls_awaiting_response.begin(); it != pull_calls_awaiting_response.end(); ++it)
    {
      if (it->call_id == message.Get<0>())
      {
        bool failed = message.Get<1>();
        core::tAbstractPort* port = core::tRuntimeEnvironment::GetInstance().GetPort(it->local_port_handle);
        if ((!failed) && port && port->IsReady() && data_ports::IsDataFlowType(port->GetDataType()))
        {
          rrlib::rtti::tType data_type;
          rrlib::time::tTimestamp timestamp;
          stream >> data_type >> timestamp;

          data_ports::tGenericPort data_port = data_ports::tGenericPort::Wrap(*port);
          data_ports::tPortDataPointer<rrlib::rtti::tGenericObject> pulled_buffer = data_port.GetUnusedBuffer();
          if (pulled_buffer->GetType() != data_type)
          {
            FINROC_LOG_PRINT(WARNING, "Port data pulled via ", port->GetQualifiedName(), " has invalid type.");
            it->promise->SetException(rpc_ports::tFutureStatus::INVALID_DATA_RECEIVED);
          }
          else
          {
            pulled_buffer.SetTimestamp(timestamp);
            pulled_buffer->Deserialize(stream);
            message.FinishDeserialize(stream);
            it->promise->SetValue(std::move(pulled_buffer));
          }
        }
        else
        {
          it->promise->SetException(rpc_ports::tFutureStatus::NO_CONNECTION);
        }
        pull_calls_awaiting_response.erase(it); // remove pull call from list
        break;
      }
    }
  }
  else if (opcode == tOpCode::SUBSCRIBE)
  {
    tSubscribeMessage message;
    message.Deserialize(stream);

    // Get or create server port
    auto it = server_port_map.find(message.Get<0>());
    if (it != server_port_map.end())
    {
      data_ports::tGenericPort port = it->second;
      tNetworkPortInfo* network_port_info = port.GetAnnotation<tNetworkPortInfo>();
      network_port_info->SetServerSideSubscriptionData(message.Get<1>(), message.Get<2>(), message.Get<3>(), message.Get<5>());

      bool push_strategy = message.Get<1>() > 0;
      bool reverse_push_strategy = message.Get<2>();
      if (port.GetWrapped()->PushStrategy() != push_strategy || port.GetWrapped()->ReversePushStrategy() != reverse_push_strategy)
      {
        // flags need to be changed
        rrlib::thread::tLock lock(GetStructureMutex(), false);
        if (lock.TryLock())
        {
          if (port.GetWrapped()->PushStrategy() != push_strategy)
          {
            port.GetWrapped()->SetPushStrategy(push_strategy);
          }
          if (port.GetWrapped()->ReversePushStrategy() != reverse_push_strategy)
          {
            port.GetWrapped()->SetReversePushStrategy(reverse_push_strategy);
          }
        }
        else
        {
          return true; // We could not obtain lock - try again later
        }
      }
    }
    else
    {
      rrlib::thread::tLock lock(GetStructureMutex(), false);
      if (lock.TryLock())
      {
        // Create server port
        core::tAbstractPort* port = core::tRuntimeEnvironment::GetInstance().GetPort(message.Get<0>());
        if ((!port) || (!port->IsReady()))
        {
          FINROC_LOG_PRINT(DEBUG_WARNING, "Port for subscription not available");
          return false;
        }

        tFlags flags = tFlag::NETWORK_ELEMENT | tFlag::VOLATILE;
        if (port->IsOutputPort())
        {
          flags |= tFlag::ACCEPTS_DATA; // create input port
        }
        else
        {
          flags |= tFlag::OUTPUT_PORT | tFlag::EMITS_DATA; // create output io port
        }
        if (send_structure_info != common::tStructureExchange::SHARED_PORTS)
        {
          flags |= tFlag::TOOL_PORT;
        }
        if (message.Get<1>() > 0)
        {
          flags |= tFlag::PUSH_STRATEGY;
        }
        if (message.Get<2>())
        {
          flags |= tFlag::PUSH_STRATEGY_REVERSE;
        }

        data_ports::tGenericPort created_port(port->GetQualifiedName().substr(1), GetServerPortsElement(), port->GetDataType(), flags);
        created_port.GetWrapped()->SetMinNetUpdateIntervalRaw(message.Get<3>());
        tNetworkPortInfo* network_port_info = new tNetworkPortInfo(*this, message.Get<4>(), message.Get<1>(), true, *created_port.GetWrapped(), message.Get<0>());
        network_port_info->SetServerSideSubscriptionData(message.Get<1>(), message.Get<2>(), message.Get<3>(), message.Get<5>());
        created_port.AddPortListenerForPointer(*network_port_info);
        created_port.Init();
        created_port.ConnectTo(port);
        server_port_map.insert(std::pair<tFrameworkElementHandle, data_ports::tGenericPort>(message.Get<0>(), created_port));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Created server port ", created_port.GetWrapped()->GetQualifiedName());
      }
      else
      {
        return true; // We could not obtain lock - try again later
      }
    }
  }
  else if (opcode == tOpCode::UNSUBSCRIBE)
  {
    tUnsubscribeMessage message;
    message.Deserialize(stream);
    auto it = server_port_map.find(message.Get<0>());
    if (it != server_port_map.end())
    {
      rrlib::thread::tLock lock(GetStructureMutex(), false);
      if (lock.TryLock())
      {
        it->second.GetWrapped()->ManagedDelete();
      }
      else
      {
        return true; // We could not obtain lock - try again later
      }
    }
    else
    {
      FINROC_LOG_PRINT(DEBUG_WARNING, "Port for unsubscribing not available");
      return false;
    }
  }
  else if (opcode == tOpCode::TYPE_UPDATE)
  {
    tTypeUpdateMessage message;
    message.Deserialize(stream, false);
    rrlib::rtti::tType type;
    stream >> type;
    remote_types.SetTime(type, stream.ReadShort());
    message.FinishDeserialize(stream);
  }
  else if (opcode == tOpCode::STRUCTURE_CREATE)
  {
    rrlib::thread::tLock lock(GetStructureMutex(), false);
    if (lock.TryLock())
    {
      tStructureCreateMessage message;
      message.Deserialize(stream, false);
      common::tFrameworkElementInfo framework_element_info;
      framework_element_info.handle = message.Get<0>();
      stream >> framework_element_info;
      message.FinishDeserialize(stream);
      AddRemotePort(framework_element_info);
    }
    else
    {
      return true; // We could not obtain lock - try again later
    }
  }
  else if (opcode == tOpCode::STRUCTURE_CHANGE)
  {
    rrlib::thread::tLock lock(GetStructureMutex(), false);
    if (lock.TryLock())
    {
      tStructureChangeMessage message;
      message.Deserialize(stream, false);
      common::tChangeablePortInfo changeable_port_info;
      stream >> changeable_port_info;
      message.FinishDeserialize(stream);

      core::tAbstractPort* port_to_change = remote_port_map[message.Get<0>()];
      if (port_to_change)
      {
        if (data_ports::IsDataFlowType(port_to_change->GetDataType()))
        {
          data_ports::common::tAbstractDataPort& data_port = static_cast<data_ports::common::tAbstractDataPort&>(*port_to_change);
          data_port.SetPushStrategy(changeable_port_info.flags.Get(tFlag::PUSH_STRATEGY));
          data_port.SetReversePushStrategy(changeable_port_info.flags.Get(tFlag::PUSH_STRATEGY_REVERSE));
          data_port.SetMinNetUpdateIntervalRaw(changeable_port_info.min_net_update_time);
          tNetworkPortInfo* network_port_info = data_port.GetAnnotation<tNetworkPortInfo>();
          if (network_port_info)
          {
            network_port_info->ChangeStrategy(changeable_port_info.strategy);
          }
        }
        else
        {
          FINROC_LOG_PRINT(WARNING, "Port to change does not have data flow type: ", port_to_change->GetQualifiedName());
        }
      }
      else
      {
        FINROC_LOG_PRINT(WARNING, "There is port to change with handle ", message.Get<0>());
      }
    }
    else
    {
      return true; // We could not obtain lock - try again later
    }
  }
  else if (opcode == tOpCode::STRUCTURE_DELETE)
  {
    rrlib::thread::tLock lock(GetStructureMutex(), false);
    if (lock.TryLock())
    {
      tStructureDeleteMessage message;
      message.Deserialize(stream);

      core::tAbstractPort* port_to_delete = remote_port_map[message.Get<0>()];
      if (port_to_delete)
      {
        tFrameworkElementHandle handle = message.Get<0>();
        remote_port_map.erase(handle);
        port_to_delete->ManagedDelete();
      }
      else
      {
        FINROC_LOG_PRINT(WARNING, "There is port to delete with handle ", message.Get<0>());
      }
    }
    else
    {
      return true; // We could not obtain lock - try again later
    }
  }
  else if (opcode == tOpCode::PEER_INFO)
  {
    tPeerInfoMessage message;
    message.Deserialize(stream, false);
    while (stream.ReadBoolean())
    {
      tPeerInfo peer(tPeerType::UNSPECIFIED);
      peer_implementation.DeserializePeerInfo(stream, peer);
      RRLIB_LOG_PRINT(DEBUG, "Deserialized peer ", peer.ToString());
      peer_implementation.ProcessIncomingPeerInfo(peer);
    }
    message.FinishDeserialize(stream);
  }

  return false;
}

void tRemotePart::ProcessStructurePacket(rrlib::serialization::tInputStream& stream)
{
  try
  {
    rrlib::rtti::tType type;
    stream >> type;
    if (type != rrlib::rtti::tDataType<std::string>())
    {
      FINROC_LOG_PRINT(ERROR, "Type encoding does not seem to work");
      return;
    }

    common::tFrameworkElementInfo info;
    while (stream.Remaining())
    {
      stream >> info.handle;
      info.Deserialize(stream);
      AddRemotePort(info);
    }
  }
  catch (const std::exception& e)
  {
    FINROC_LOG_PRINT(ERROR, "Error processing structure packet:", e);
  }
}

void tRemotePart::RemoveConnection(tConnection& connection)
{
  if (management_connection.get() == &connection)
  {
    management_connection.reset();

    // Delete all calls
    not_ready_calls.clear();
    calls_awaiting_response.clear();

    // Delete all remote ports
    for (auto it = ChildrenBegin(); it != ChildrenEnd(); ++it)
    {
      it->ManagedDelete();
    }
    global_links = NULL;
    server_ports = NULL;
  }
  if (express_connection.get() == &connection)
  {
    express_connection.reset();
  }
  if (bulk_connection.get() == &connection)
  {
    bulk_connection.reset();
  }
  if (peer_info.connected)
  {
    peer_info.last_connection = rrlib::time::Now(false);
  }
  if (peer_info.connected)
  {
    FINROC_LOG_PRINT(DEBUG, "Disconnected from ", this->GetName());
  }
  peer_info.connected = false;
}

void tRemotePart::RpcPortsDeleted(std::vector<core::tFrameworkElement::tHandle>& deleted_ports)
{
  if (management_connection)
  {
    management_connection->RpcPortsDeleted(deleted_ports);
  }
  if (express_connection.get() && express_connection.get() != management_connection.get())
  {
    express_connection->RpcPortsDeleted(deleted_ports);
  }
  if (bulk_connection.get() && bulk_connection.get() != express_connection.get() && bulk_connection.get() != management_connection.get())
  {
    bulk_connection->RpcPortsDeleted(deleted_ports);
  }
}

void tRemotePart::SendCall(tCallPointer& call_to_send, const rrlib::time::tTimestamp& time_now)
{
  if (!call_to_send->ReadyForSending())
  {
    //FINROC_LOG_PRINT(ERROR, "Emplacing ", call_to_send.get());
    not_ready_calls.emplace_back(std::move(call_to_send));
  }
  else
  {
    SendCallImplementation(call_to_send, time_now);
  }
}

void tRemotePart::SendCallImplementation(tCallPointer& call_to_send, const rrlib::time::tTimestamp& time_now)
{
  if (express_connection)
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Sending Call ", make_builder::GetEnumString(call_to_send->GetCallType()));
    bool expects_response = call_to_send->ExpectsResponse();
    if (expects_response)
    {
      call_to_send->SetCallId(next_call_id);
      next_call_id++;
    }
    tRPCCall::Serialize(false, express_connection->CurrentWriteStream(), call_to_send->GetRemotePortHandle(), call_to_send->GetCallType());
    call_to_send->GetCall()->Serialize(express_connection->CurrentWriteStream());
    tRPCCall::FinishMessage(express_connection->CurrentWriteStream());
    if (expects_response)
    {
      rrlib::time::tDuration timeout = call_to_send->ResponseTimeout();
      calls_awaiting_response.emplace_back(time_now + timeout, std::move(call_to_send));
    }
  }
  else
  {
    call_to_send.reset();
  }
}

void tRemotePart::SendPendingMessages(const rrlib::time::tTimestamp& time_now)
{
  for (auto it = not_ready_calls.begin(); it < not_ready_calls.end(); ++it)
  {
    if ((*it)->ReadyForSending())
    {
      tCallPointer call_pointer = std::move(*it);
      SendCallImplementation(call_pointer, time_now);
      it = not_ready_calls.erase(it);
    }
  }
  for (auto it = calls_awaiting_response.begin(); it < calls_awaiting_response.end(); ++it)
  {
    if (time_now > it->first) // Did call time out?
    {
      it = calls_awaiting_response.erase(it);
    }
  }
  for (auto it = pull_calls_awaiting_response.begin(); it < pull_calls_awaiting_response.end(); ++it)
  {
    if (time_now > it->timeout_time) // Did call time out?
    {
      it = pull_calls_awaiting_response.erase(it);
    }
  }

  if (management_connection)
  {
    management_connection->SendPendingMessages(time_now);
  }
  if (express_connection.get() && express_connection.get() != management_connection.get())
  {
    express_connection->SendPendingMessages(time_now);
  }
  if (bulk_connection.get() && bulk_connection.get() != express_connection.get() && bulk_connection.get() != management_connection.get())
  {
    bulk_connection->SendPendingMessages(time_now);
  }
}

void tRemotePart::SendPullRequest(tPullCallInfo& pull_call_info)
{
  // We do this here, because this is the TCP thread now (and next_call_id is not thread-safe)
  pull_call_info.call_id = next_call_id;
  next_call_id++;
  pull_calls_awaiting_response.push_back(pull_call_info);

  // Send call
  tPullCall::Serialize(true, express_connection->CurrentWriteStream(), pull_call_info.remote_port_handle,
                       pull_call_info.call_id, rrlib::serialization::tDataEncoding::BINARY);
  this->SendPendingMessages(rrlib::time::Now(true));
}

void tRemotePart::SendResponse(typename tResponseSender::tCallPointer && response_to_send)
{
  if (response_to_send->ReadyForSending())
  {
    rrlib::time::tTimestamp time_now = rrlib::time::Now();
    SendCallImplementation(response_to_send, time_now);
    SendPendingMessages(time_now);
  }
  else
  {
    not_ready_calls.emplace_back(std::move(response_to_send));
  }
}

void tRemotePart::SetDesiredStructureInfo(common::tStructureExchange send_structure_info)
{
  if (send_structure_info == common::tStructureExchange::NONE)
  {
    return;
  }
  if (this->send_structure_info != common::tStructureExchange::NONE && this->send_structure_info != send_structure_info)
  {
    FINROC_LOG_PRINT(WARNING, "Desired structure info already set to ", make_builder::GetEnumString(this->send_structure_info), ". This is likely to cause trouble.");
  }
  this->send_structure_info = send_structure_info;
}

tRemotePart::tPullCallInfo::tPullCallInfo(tRemotePart& remote_part, data_ports::common::tAbstractDataPort& local_port) :
  call_id(0),
  promise(new rpc_ports::tPromise<data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject>>()), // TODO: optimize (avoid memory allocation)
  local_port_handle(local_port.GetHandle()),
  remote_port_handle(0),
  timeout_time(rrlib::time::Now(false) + data_ports::cPULL_TIMEOUT),
  remote_part(&remote_part)
{
  tNetworkPortInfo* network_port_info = local_port.GetAnnotation<tNetworkPortInfo>();
  assert(network_port_info && "Can only handle pull requests from ports with network port info");
  remote_port_handle = network_port_info->GetRemoteHandle();
}

void tRemotePart::tPullCallInfo::operator()()
{
  remote_part->SendPullRequest(*this);
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
