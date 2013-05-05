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
/*!\file    plugins/tcp/internal/tConnection.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tConnection.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/asio.hpp>
#include "core/tRuntimeEnvironment.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/common/tNetworkUpdateTimeSettings.h"
#include "plugins/tcp/internal/tNetworkPortInfo.h"
#include "plugins/tcp/internal/tPeerImplementation.h"
#include "plugins/tcp/internal/tRemotePart.h"

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
static const size_t c1ST_INITIAL_MESSAGE_LENGTH = strlen(cGREET_MESSAGE) + 7;

static const size_t cINITIAL_READ_BUFFER_SIZE = 32768;
static const size_t cINITIAL_WRITE_BUFFER_SIZE = 32768;

static const size_t cMAX_MESSAGE_BATCH_SIZE = 300000000; // more than 30 MB

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

/*!
 * Writes things necessary for connection initialization to socket.
 * Connections are initialized as follows:
 *
 * 1st part: [GREET_MESSAGE][2 byte protocol version][4 byte 2nd part message length]
 * 2nd part: [my UUID][peer type][peer name][structure exchange][connection flags][your address]
 */
class tInitialWriteHandler
{
public:

  tInitialWriteHandler(std::shared_ptr<tConnection>& connection) :
    initial_message_buffers(new std::array<rrlib::serialization::tStackMemoryBuffer<200>, 2>()),
    initial_message_asio_buffers(),
    connection(connection)
  {
    rrlib::serialization::tOutputStream stream1((*initial_message_buffers)[0]);
    rrlib::serialization::tOutputStream stream2((*initial_message_buffers)[1]);
    stream1 << cGREET_MESSAGE;
    stream1.WriteShort(cPROTOCOL_VERSION);
    tConnectionInitMessage::Serialize(true, stream2, connection->peer.GetPeerInfo().uuid, connection->peer.GetPeerInfo().peer_type,
                                      connection->peer.GetPeerInfo().name, common::tStructureExchange::SHARED_PORTS, connection->flags, connection->socket->remote_endpoint().address());
    stream2.Close();
    stream1.Close();

    initial_message_asio_buffers[0] = boost::asio::const_buffer((*initial_message_buffers)[0].GetBufferPointer(0), (*initial_message_buffers)[0].GetSize());
    initial_message_asio_buffers[1] = boost::asio::const_buffer((*initial_message_buffers)[1].GetBufferPointer(0), (*initial_message_buffers)[1].GetSize());

    assert((*initial_message_buffers)[0].GetSize() == c1ST_INITIAL_MESSAGE_LENGTH - 4);

    boost::asio::async_write(*(connection->socket), initial_message_asio_buffers, *this);
  }

  void operator()(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      connection->Close();
    }
    else
    {
      connection->initial_writing_complete = true;
      if (connection->initial_reading_complete && connection->initial_writing_complete)
      {
        connection->DoInitialStructureExchange(connection);
      }
    }
  }

private:

  std::shared_ptr<std::array<rrlib::serialization::tStackMemoryBuffer<200>, 2>> initial_message_buffers;
  std::array<boost::asio::const_buffer, 2> initial_message_asio_buffers;
  std::shared_ptr<tConnection> connection;
};

/*!
 * Reads and interprets initial message from connection partner
 */
class tInitialReadHandler
{
public:

  tInitialReadHandler(std::shared_ptr<tConnection>& connection) :
    initial_message_buffer(new std::array<uint8_t, 300>()),
    connection(connection),
    first_message_received(false)
  {
    boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(initial_message_buffer->begin(), c1ST_INITIAL_MESSAGE_LENGTH), *this);
  }

  void operator()(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      connection->Close();
      return;
    }
    try
    {
      rrlib::serialization::tMemoryBuffer read_buffer(initial_message_buffer->begin(), bytes_transferred);
      rrlib::serialization::tInputStream stream(read_buffer);
      if (!first_message_received)
      {
        if (strcmp(stream.ReadString().c_str(), cGREET_MESSAGE) != 0)
        {
          connection->Close();
          return;
        }
        if (stream.ReadShort() != cPROTOCOL_VERSION)
        {
          connection->Close();
          return;
        }
        int size = stream.ReadInt();
        if (size > 300)
        {
          connection->Close();
          return;
        }

        first_message_received = true;

        boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(initial_message_buffer->begin(), size), *this);
      }
      else if (!connection->closed)
      {
        tConnectionInitMessage message;
        message.Deserialize(stream);

        if (message.Get<3>() != common::tStructureExchange::SHARED_PORTS && (!connection->peer.ServesStructure()))
        {
          connection->Close(); // Not serving structure yet
          return;
        }

        connection->peer.AddAddress(message.Get<5>());
        connection->remote_part = connection->peer.GetRemotePart(message.Get<0>(), message.Get<1>(), message.Get<2>(),
                                  connection->socket->remote_endpoint().address(), connection->never_forget);

        connection->flags |= message.Get<4>();

        if (!connection->remote_part->AddConnection(connection))
        {
          connection->Close(); // we already have a connection of this type
          return;
        }
        connection->remote_part->SetDesiredStructureInfo(message.Get<3>());

        connection->initial_reading_complete = true;
        if (connection->initial_reading_complete && connection->initial_writing_complete)
        {
          connection->DoInitialStructureExchange(connection);
        }
      }
    }
    catch (const std::exception& exception)
    {
      FINROC_LOG_PRINT_STATIC(WARNING, "Rejected TCP connection because invalid connection initialization data was received.");
      connection->Close();
    }
  }

private:

  std::shared_ptr<std::array<uint8_t, 300>> initial_message_buffer;
  std::shared_ptr<tConnection> connection;
  bool first_message_received;
};

/*!
 * Writes a complete batch of message to stream
 */
class tMessageBatchWriteHandler
{
public:
  tMessageBatchWriteHandler(std::shared_ptr<tConnection>& connection) : connection(connection)
  {
    boost::asio::async_write(*(connection->socket), boost::asio::const_buffers_1(connection->back_buffer.GetBufferPointer(0), connection->back_buffer.GetSize()), *this);
  }

  void operator()(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error || bytes_transferred != connection->back_buffer.GetSize())
    {
      connection->Close();
    }
    else
    {
      connection->writing_back_buffer_to_stream = false;
    }
  }

private:
  std::shared_ptr<tConnection> connection;
};

/*!
 * Reads a complete batch of messages from stream and processes them
 */
class tMessageBatchReadHandler
{
public:

  tMessageBatchReadHandler(std::shared_ptr<tConnection>& connection) : connection(connection), continue_at(0)
  {
    // read size of next structure buffer
    boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(connection->read_size_buffer.GetPointer(), 4), *this);
  }

  void operator()(const boost::system::error_code& error)
  {
    if (error)
    {
      connection->Close();
      return;
    }
    ProcessMessageBatch();
  }

  void operator()(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Closing connection: ", error.message());
      connection->Close();
      return;
    }
    size_t bytes_required = connection->bytes_to_read ? connection->bytes_to_read : 4;
    if (bytes_transferred != bytes_required)
    {
      FINROC_LOG_PRINT(DEBUG_WARNING, "Not enough bytes transferred");
      connection->Close();
      return;
    }

    if (!connection->bytes_to_read)
    {
      size_t bytes = connection->GetBytesToRead();

      if (connection->read_buffer.Capacity() < bytes)
      {
        if (bytes > cMAX_MESSAGE_BATCH_SIZE)
        {
          FINROC_LOG_PRINT(WARNING, "Client wanted to send packet of size ", bytes, "! Closing connection.");
          connection->Close();
          return;
        }

        connection->read_buffer = rrlib::serialization::tFixedBuffer(bytes + 5000);
      }
      if (!connection->closed)
      {
        boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(connection->read_buffer.GetPointer(), bytes), *this);
      }
    }
    else
    {
      ProcessMessageBatch();
    }
  }

  void ProcessMessageBatch()
  {
    continue_at = connection->ProcessMessageBatch(continue_at);

    if (continue_at == 0)
    {
      // read size of next structure buffer
      connection->bytes_to_read = 0;
      boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(connection->read_size_buffer.GetPointer(), 4), *this);
    }
    else
    {
      // Defer processing 2ms
      connection->defer_read_timer.expires_from_now(boost::posix_time::milliseconds(2));
      connection->defer_read_timer.async_wait(*this);
    }
  }

private:

  std::shared_ptr<tConnection> connection;
  size_t continue_at;
};

/*!
 * Writes structure to stream
 * TODO: continue
 */
class tStructureWriteHandler
{
public:

  tStructureWriteHandler(std::shared_ptr<tConnection> connection) :
    connection(connection),
    buffer(),
    ready_after_write(false)
  {
    if (connection->remote_part->GetDesiredStructureInfo() == common::tStructureExchange::SHARED_PORTS)
    {
      buffer.reset(new rrlib::serialization::tMemoryBuffer(0));
      *buffer = connection->peer.SerializeSharedPorts(connection->remote_types);
      ready_after_write = true;
      boost::asio::async_write(*(connection->socket), boost::asio::const_buffers_1(buffer->GetBufferPointer(0), buffer->GetSize()), *this);
    }
    else
    {
      SerializeNextElements();
    }
  }

  void operator()(const boost::system::error_code& error)
  {
    if (error)
    {
      connection->Close();
      return;
    }
    SerializeNextElements();
  }

  void operator()(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error || bytes_transferred != buffer->GetSize())
    {
      connection->Close();
    }
    else if (ready_after_write)
    {
      connection->initial_structure_writing_complete = true;
      if (connection->initial_structure_reading_complete)
      {
        connection->ready = true;
        tMessageBatchReadHandler handler(connection);
        return;
      }
    }
    else
    {
      SerializeNextElements();
    }
  }

  void SerializeNextElements()
  {
    typedef core::tFrameworkElement::tHandle tHandle;
    assert(connection->remote_part->GetDesiredStructureInfo() == common::tStructureExchange::FINSTRUCT ||
           connection->remote_part->GetDesiredStructureInfo() == common::tStructureExchange::COMPLETE_STRUCTURE);
    rrlib::thread::tLock lock(core::tRuntimeEnvironment::GetInstance().GetStructureMutex(), false);
    if (lock.TryLock())
    {
      connection->peer.ProcessRuntimeChangeEvents();  // to make sure we don't get any shared port events twice
      core::tFrameworkElement* framework_element_buffer[1000];
      size_t element_count = 0;
      if (connection->framework_elements_in_full_structure_exchange_sent_until_handle <= std::numeric_limits<tHandle>::max())
      {
        element_count = core::tRuntimeEnvironment::GetInstance().GetAllElements(framework_element_buffer, 1000,
                        connection->framework_elements_in_full_structure_exchange_sent_until_handle);
      }
      if (!buffer)
      {
        buffer.reset(new rrlib::serialization::tMemoryBuffer(element_count * 200 + 4));
      }
      rrlib::serialization::tOutputStream stream(*buffer, connection->remote_types);
      stream.WriteInt(0); // placeholder for size
      std::string temp_buffer;
      for (size_t i = 0; i < element_count; i++)
      {
        bool relevant = (connection->remote_part->GetDesiredStructureInfo() == common::tStructureExchange::FINSTRUCT) ||
                        (!framework_element_buffer[i]->GetFlag(core::tFrameworkElement::tFlag::NETWORK_ELEMENT));
        if (relevant)
        {
          stream.WriteInt(framework_element_buffer[i]->GetHandle());
          common::tFrameworkElementInfo::Serialize(stream, *framework_element_buffer[i], connection->remote_part->GetDesiredStructureInfo(), temp_buffer);
          FINROC_LOG_PRINT(DEBUG_VERBOSE_2, "Serializing ", framework_element_buffer[i]->GetQualifiedName());
        }
        connection->framework_elements_in_full_structure_exchange_sent_until_handle = framework_element_buffer[i]->GetHandle() + 1;
      }
      stream.Close();
      size_t payload_size = buffer->GetSize() - 4;
      buffer->GetBuffer()->PutInt(0, payload_size);
      ready_after_write = (payload_size == 0);
      boost::asio::async_write(*(connection->socket), boost::asio::const_buffers_1(buffer->GetBufferPointer(0), buffer->GetSize()), *this);
    }
    else
    {
      connection->defer_write_timer.expires_from_now(boost::posix_time::milliseconds(2));
      connection->defer_write_timer.async_wait(*this);
    }
  }

private:

  std::shared_ptr<tConnection> connection;
  std::shared_ptr<rrlib::serialization::tMemoryBuffer> buffer;
  bool ready_after_write;
};

/*!
 * Reads and interprets initial message from connection partner
 */
class tStructureReadHandler
{
public:

  tStructureReadHandler(std::shared_ptr<tConnection> connection) : connection(connection), read_buffer()
  {
    // read size of next structure buffer
    boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(connection->read_size_buffer.GetPointer(), 4), *this);
  }

  void operator()(const boost::system::error_code& error)
  {
    if (error)
    {
      connection->Close();
      return;
    }
    ProcessPacket();
  }

  void operator()(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      connection->Close();
      return;
    }
    size_t bytes_required = connection->bytes_to_read ? connection->bytes_to_read : 4;
    if (bytes_transferred != bytes_required)
    {
      FINROC_LOG_PRINT(DEBUG_WARNING, "Not enough bytes transferred");
      connection->Close();
      return;
    }

    if (!connection->bytes_to_read)
    {
      size_t bytes = connection->GetBytesToRead();
      if (bytes == 0)
      {
        // ready
        connection->initial_structure_reading_complete = true;
        if (connection->initial_structure_writing_complete)
        {
          connection->ready = true;
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Reading structure complete, waiting for message batch");
          tMessageBatchReadHandler handler(connection);
        }
        return;
      }

      if (bytes > cMAX_MESSAGE_BATCH_SIZE)
      {
        FINROC_LOG_PRINT(WARNING, "Client wanted to send packet of size ", bytes, "! Closing connection.");
        connection->Close();
        return;
      }

      if ((!read_buffer) || read_buffer->Capacity() < bytes)
      {
        read_buffer.reset(new rrlib::serialization::tFixedBuffer(bytes));
      }
      boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(read_buffer->GetPointer(), bytes), *this);
    }
    else
    {
      ProcessPacket();
    }
  }

  void ProcessPacket()
  {
    // try to create structure
    rrlib::thread::tLock lock(connection->remote_part->GetStructureMutex(), false);
    if (lock.TryLock())
    {
      rrlib::serialization::tMemoryBuffer packet_buffer(read_buffer->GetPointer(), connection->bytes_to_read);
      rrlib::serialization::tInputStream stream(packet_buffer, connection->remote_types);
      connection->remote_part->ProcessStructurePacket(stream);
      lock.Unlock();

      // read size of next structure buffer
      connection->bytes_to_read = 0;
      boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(connection->read_size_buffer.GetPointer(), 4), *this);
    }
    else
    {
      // Try again in 2ms
      connection->defer_read_timer.expires_from_now(boost::posix_time::milliseconds(2));
      connection->defer_read_timer.async_wait(*this);
    }
  }

private:

  std::shared_ptr<tConnection> connection;
  std::shared_ptr<rrlib::serialization::tFixedBuffer> read_buffer;
};


tConnection::tConnection(tPeerImplementation& peer, std::shared_ptr<boost::asio::ip::tcp::socket>& socket, int flags,
                         std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator, bool never_forget) :
  peer(peer),
  flags(flags),
  socket(socket),
  closed(false),
  remote_part(NULL),
  initial_reading_complete(false),
  initial_writing_complete(false),
  initial_structure_reading_complete(false),
  initial_structure_writing_complete(false),
  ready(false),
  read_size_buffer(4),
  read_buffer(cINITIAL_READ_BUFFER_SIZE),
  bytes_to_read(0),
  front_buffer(cINITIAL_WRITE_BUFFER_SIZE),
  back_buffer(cINITIAL_WRITE_BUFFER_SIZE),
  writing_back_buffer_to_stream(false),
  remote_types(),
  current_write_stream(front_buffer, remote_types),
  active_connect_indicator(active_connect_indicator),
  never_forget(never_forget),
  defer_read_timer(peer.IOService()),
  defer_write_timer(peer.IOService()),
  last_ack_request_index(-1),
  last_acknowledged_packet(0),
  next_packet_index(1),
  sent_bulk_packets(0),
  sent_packet_data(),
  framework_elements_in_full_structure_exchange_sent_until_handle(0),
  rpc_call_buffer_pools()
{
  current_write_stream.WriteInt(0); // Placeholder for size
  current_write_stream.WriteShort(0); // Placeholder for ack requests
  current_write_stream.WriteShort(0); // Placeholder for acks
  sent_packet_data.fill(tSentPacketData(rrlib::time::cNO_TIME, 0));
}

tConnection::~tConnection()
{}

void tConnection::Close()
{
  if (!closed)
  {
    try
    {
      // socket->shutdown(); TODO: Add with newer boost version (?)
      socket->close();
    }
    catch (const std::exception& ex)
    {
      FINROC_LOG_PRINT(WARNING, "Closing connection failed: ", ex);
    }
    closed = true;
    if (remote_part)
    {
      remote_part->RemoveConnection(*this);
    }
  }
}

void tConnection::DoInitialStructureExchange(std::shared_ptr<tConnection> connection)
{
  if ((flags & static_cast<int>(tConnectionFlag::MANAGEMENT_DATA)))
  {
    assert(this == remote_part->GetManagementConnection().get());
    tStructureReadHandler structure_read_handler(remote_part->GetManagementConnection());
    if (remote_part->GetDesiredStructureInfo() != common::tStructureExchange::NONE)
    {
      tStructureWriteHandler structure_write_handler(remote_part->GetManagementConnection());
    }
    else
    {
      initial_structure_writing_complete = true;
    }
  }
  else
  {
    initial_structure_reading_complete = true;
    initial_structure_writing_complete = true;
    ready = true;
    tMessageBatchReadHandler handler(connection);
  }
}

size_t tConnection::GetBytesToRead()
{
  rrlib::serialization::tMemoryBuffer buffer(read_size_buffer.GetPointer(), read_size_buffer.Capacity());
  rrlib::serialization::tInputStream stream(buffer);
  bytes_to_read = stream.ReadInt();
  return bytes_to_read;
}

void tConnection::InitConnection(tPeerImplementation& peer, std::shared_ptr<boost::asio::ip::tcp::socket>& socket, int flags,
                                 std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator, bool never_forget)
{
  std::shared_ptr<tConnection> connection(new tConnection(peer, socket, flags, active_connect_indicator, never_forget));

  tInitialWriteHandler write_handler(connection);
  tInitialReadHandler read_handler(connection);
}

size_t tConnection::ProcessMessageBatch(size_t start_at)
{
  try
  {
    rrlib::serialization::tMemoryBuffer mem_buffer(read_buffer.GetPointer(), GetBytesToRead());
    rrlib::serialization::tInputStream stream(mem_buffer);

    // process acknowledging
    if (start_at == 0)
    {
      // packet acknowledgement request
      int16_t ack_request_index = stream.ReadShort();
      if (ack_request_index >= 0)
      {
        last_ack_request_index = ack_request_index;
      }

      // packet acknowledgements
      int16_t acknowledgement = stream.ReadShort();
      if (acknowledgement >= 0)
      {
        last_acknowledged_packet = acknowledgement;
      }
    }
    else
    {
      stream.Skip(start_at);
    }

    while (stream.MoreDataAvailable())
    {
      size_t command_start_position = static_cast<size_t>(stream.GetAbsoluteReadPosition());
      tOpCode op_code = stream.ReadEnum<tOpCode>();
      if (op_code >= tOpCode::OTHER)
      {
        FINROC_LOG_PRINT(WARNING, "Received corrupted TCP message batch. Invalid opcode. Skipping.");
        return 0;
      }
      size_t message_size = message_size_for_opcodes[static_cast<size_t>(op_code)].ReadMessageSize(stream);
      if (message_size == 0 || message_size > stream.Remaining())
      {
        FINROC_LOG_PRINT(WARNING, "Received corrupted TCP message batch. Invalid message size: ", message_size, ". Skipping.");
        return 0;
      }
      rrlib::serialization::tMemoryBuffer message_buffer(mem_buffer.GetBufferPointer(static_cast<size_t>(stream.GetAbsoluteReadPosition())), message_size);
      stream.Skip(message_size);

      try
      {
        bool defer = remote_part->ProcessMessage(op_code, message_buffer, remote_types, *this);
        if (defer)
        {
          return command_start_position;
        }
      }
      catch (const std::exception& ex)
      {
        FINROC_LOG_PRINT(WARNING, "Failed to deserialize message of type ", make_builder::GetEnumString(op_code), ". Skipping. Cause: ", ex);
      }
    }
  }
  catch (const std::exception& ex)
  {
    FINROC_LOG_PRINT(WARNING, "Error processing message batch: ", ex, ". Skipping.");
  }
  return 0;
}

void tConnection::RpcPortsDeleted(std::vector<core::tFrameworkElement::tHandle>& deleted_ports)
{
  for (auto it = deleted_ports.begin(); it != deleted_ports.end(); ++it)
  {
    rpc_call_buffer_pools.erase(*it);
  }
}

void tConnection::SendPendingMessages(const rrlib::time::tTimestamp& time_now)
{
  if (writing_back_buffer_to_stream) // back buffer is still written - we cannot write anything else during this time
  {
    return;
  }

  // Determine whether more packages can be sent (ugly: counter wrap-arounds)
  int non_acknowledged_express_packets = (next_packet_index - 1) - last_acknowledged_packet;
  if (non_acknowledged_express_packets < 0)
  {
    non_acknowledged_express_packets += 0x8000;
  }
  assert(non_acknowledged_express_packets >= 0 && non_acknowledged_express_packets <= tSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS);
  bool more_express_packets_allowed = non_acknowledged_express_packets < tSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS;

  tSentPacketData& last_acknowledged = sent_packet_data[last_acknowledged_packet % sent_packet_data.size()];
  uint32_t non_acknowledged_bulk_packets = sent_bulk_packets - last_acknowledged.second;
  bool more_bulk_packets_allowed = non_acknowledged_bulk_packets < tSettings::GetInstance().max_not_acknowledged_packets_bulk.Get();

  // Send new port data
  if (more_express_packets_allowed && (flags & static_cast<int>(tConnectionFlag::EXPRESS_DATA)))
  {
    if (!remote_part->PortsWithExpressDataToSend().empty())
    {
      //FINROC_LOG_PRINT(DEBUG_WARNING, "sdfsdf");
      SendPortData(remote_part->PortsWithExpressDataToSend(), time_now);
    }
  }
  if (more_bulk_packets_allowed && (flags & static_cast<int>(tConnectionFlag::BULK_DATA)))
  {
    if (!remote_part->PortsWithBulkDataToSend().empty())
    {
      if (SendPortData(remote_part->PortsWithBulkDataToSend(), time_now))
      {
        sent_bulk_packets++;
      }
    }
  }


  // Send packet
  current_write_stream.Flush();
  if (front_buffer.GetSize() > 8 || last_ack_request_index >= 0)
  {

    // update internal acknowledgement management variables
    bool data_packet = more_express_packets_allowed && front_buffer.GetSize() > 8;
    int16_t ack_request = -1;
    if (data_packet)
    {
      ack_request = next_packet_index;
      tSentPacketData& current_packet_data = sent_packet_data[next_packet_index % sent_packet_data.size()];
      current_packet_data.first = time_now;
      current_packet_data.second = sent_bulk_packets;

      next_packet_index++;
      if (next_packet_index < 0)
      {
        next_packet_index = 0;
      }
    }

    // send a packet
    front_buffer.GetBuffer()->PutInt(0, front_buffer.GetSize() - 4); // put size
    front_buffer.GetBuffer()->PutShort(4, ack_request); // put ack request
    front_buffer.GetBuffer()->PutShort(6, last_ack_request_index); // put ack response
    last_ack_request_index = -1;
    std::swap(front_buffer, back_buffer);

    current_write_stream.Reset(front_buffer);
    current_write_stream.WriteInt(0); // Placeholder for size
    current_write_stream.WriteShort(0); // Placeholder for ack requests
    current_write_stream.WriteShort(0); // Placeholder for acks
    SerializeAnyNewTypes();

    // Obtain shared pointer on connection: TODO could be significantly less ugly
    std::shared_ptr<tConnection> connection;
    if (flags & static_cast<int>(tConnectionFlag::MANAGEMENT_DATA))
    {
      connection = remote_part->GetManagementConnection();
    }
    else if (flags & static_cast<int>(tConnectionFlag::EXPRESS_DATA))
    {
      connection = remote_part->GetExpressConnection();
    }
    else if (flags & static_cast<int>(tConnectionFlag::BULK_DATA))
    {
      connection = remote_part->GetBulkConnection();
    }

    if (connection.get() == this)
    {
      writing_back_buffer_to_stream = true;
      tMessageBatchWriteHandler write_handler(connection);
    }
    else
    {
      FINROC_LOG_PRINT(ERROR, "Could not get shared pointer on connection");
    }
  }
}

bool tConnection::SendPortData(std::vector<tNetworkPortInfo*>& port_list, const rrlib::time::tTimestamp& time_now)
{
  bool result = false;
  size_t keep_count = 0;
  for (auto it = port_list.begin(); it != port_list.end(); ++it)
  {
    tNetworkPortInfo& port = **it;
    if (port.GetLastUpdate() + port.GetUpdateInterval() <= time_now)
    {
      port.WriteDataBuffersToStream(current_write_stream, time_now);
      result = true;
    }
    else
    {
      port_list[keep_count] = *it; // keep port in list for later sending
      keep_count++;
    }
  }

  port_list.resize(keep_count, NULL);

  return result;
}

void tConnection::SendStructureChange(const tSerializedStructureChange& structure_change, common::tStructureExchange structure_exchange_level)
{
  if (initial_reading_complete && initial_writing_complete)
  {
    if ((structure_exchange_level == common::tStructureExchange::SHARED_PORTS) || initial_structure_writing_complete ||
        structure_change.GetLocalHandle() < framework_elements_in_full_structure_exchange_sent_until_handle)
    {
      if (initial_structure_writing_complete)
      {
        // check whether we need to serialize further types
        SerializeAnyNewTypes();
      }
      structure_change.WriteToStream(current_write_stream, structure_exchange_level);
    }
  }
}

void tConnection::SerializeAnyNewTypes()
{
  if (remote_types.TypeUpdateNecessary())
  {
    tTypeUpdateMessage message;
    message.Serialize(false, current_write_stream);
    current_write_stream << rrlib::rtti::tType();
    current_write_stream.WriteShort(common::tNetworkUpdateTimeSettings::GetInstance().default_minimum_network_update_time.Get());
    message.FinishMessage(current_write_stream);
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
