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
typedef network_transport::runtime_info::tStructureExchange tStructureExchange;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static const size_t c1ST_INITIAL_MESSAGE_LENGTH = strlen(cGREET_MESSAGE) + 7;

static const size_t cINITIAL_READ_BUFFER_SIZE = 32768;
static const size_t cINITIAL_WRITE_BUFFER_SIZE = 32768;

static const size_t cMAX_MESSAGE_BATCH_SIZE = 300000000; // more than 30 MB

/** Serialization info for initial message */
static const rrlib::serialization::tSerializationInfo cINITIAL_SERIALIZATION_INFO(0, rrlib::serialization::tRegisterEntryEncoding::UID, network_transport::runtime_info::cDEBUG_PROTOCOL);

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

namespace
{

rrlib::serialization::tMemoryBuffer SerializeSharedPorts(const network_transport::generic_protocol::tLocalRuntimeInfo::tSharedPortInfo& info, rrlib::serialization::tOutputStream& stream_prototype)
{
  rrlib::thread::tLock lock(core::tRuntimeEnvironment::GetInstance().GetStructureMutex());
  rrlib::serialization::tMemoryBuffer buffer(info.size() * 200);
  rrlib::serialization::tOutputStream stream(buffer, stream_prototype);
  stream.WriteInt(0); // Placeholder for size
  stream << rrlib::rtti::tDataType<std::string>(); // write a data type for initialization (only necessary in legacy mode - kept for backward-compatibility)
  for (auto & entry : info)
  {
    stream << entry.second;
  }
  stream.WriteInt(0); // size of next packet
  stream.Close();
  buffer.GetBuffer().PutInt(0, buffer.GetSize() - 8);
  return buffer;
}

}

/*!
 * Writes things necessary for connection initialization to socket.
 * Connections are initialized as follows:
 *
 * 1st part: [GREET_MESSAGE][2 byte protocol version major][4 byte 2nd part message length]
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
    rrlib::serialization::tOutputStream stream2((*initial_message_buffers)[1], cINITIAL_SERIALIZATION_INFO);
    stream1 << cGREET_MESSAGE;
    stream1.WriteShort(network_transport::generic_protocol::cPROTOCOL_VERSION_MAJOR);
    uint32_t flags = connection->flags | (core::internal::tFrameworkElementRegister::cSTAMP_BIT_WIDTH << 8) | ((network_transport::generic_protocol::cPROTOCOL_VERSION_MINOR & 0xFFFF) << 16) | (connection->peer.par_debug_tcp.Get() ? 0 : (static_cast<uint>(tConnectionFlag::NO_DEBUG)));
    tConnectionInitMessage::Serialize(true, false, stream2, connection->peer.GetPeerInfo().uuid, connection->peer.GetPeerInfo().peer_type,
                                      connection->peer.GetPeerInfo().name, tStructureExchange::SHARED_PORTS, flags, connection->socket->remote_endpoint().address());

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
      connection->SharedConnectionInfo().initial_writing_complete = true;
      if (connection->SharedConnectionInfo().initial_reading_complete && connection->SharedConnectionInfo().initial_writing_complete)
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
      rrlib::serialization::tInputStream stream(read_buffer, cINITIAL_SERIALIZATION_INFO);
      if (!first_message_received)
      {
        if (strcmp(stream.ReadString().c_str(), cGREET_MESSAGE) != 0)
        {
          connection->Close();
          return;
        }
        if (stream.ReadShort() != network_transport::generic_protocol::cPROTOCOL_VERSION_MAJOR)
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
      else if (!connection->IsClosed())
      {
        tConnectionInitMessage message;
        message.Deserialize(stream);

        if (!network_transport::generic_protocol::tLocalRuntimeInfo::IsServingStructure())
        {
          connection->Close(); // Not serving structure yet
          return;
        }

        connection->peer.AddAddress(message.Get<5>());
        connection->SharedConnectionInfo().remote_runtime = connection->peer.GetRemoteRuntime(connection, message.Get<0>(), message.Get<1>(), message.Get<2>(),
            connection->socket->remote_endpoint().address(), connection->never_forget);
        connection->flags |= message.Get<4>() & 0xFF;

        // Setup shared connection info stream prototypes
        bool java_partner = connection->flags & static_cast<uint>(tConnectionFlag::JAVA_PEER);
        bool primary_connection = connection->flags & static_cast<uint>(tConnectionFlag::PRIMARY_CONNECTION);
        bool express_only_connection = (connection->flags & static_cast<uint>(tConnectionFlag::EXPRESS_DATA)) && (!primary_connection);
        uint serialization_revision = std::min<uint>(network_transport::generic_protocol::cPROTOCOL_VERSION_MINOR, static_cast<uint>(message.Get<4>()) >> 16);
        bool debug_tcp = serialization_revision || (!((!connection->peer.par_debug_tcp.Get()) || (connection->flags & static_cast<uint>(tConnectionFlag::NO_DEBUG))));
        rrlib::serialization::tSerializationInfo serialization_info(serialization_revision, rrlib::serialization::tRegisterEntryEncoding::UID, static_cast<int>(message.Get<3>()) | (java_partner ? network_transport::runtime_info::cJAVA_CLIENT : 0) | (debug_tcp ? network_transport::runtime_info::cDEBUG_PROTOCOL : 0));
        rrlib::serialization::tSerializationInfo deserialization_info(serialization_revision, rrlib::serialization::tRegisterEntryEncoding::UID, static_cast<int>(tStructureExchange::SHARED_PORTS) | (debug_tcp ? network_transport::runtime_info::cDEBUG_PROTOCOL : 0));
        serialization_info.SetRegisterEntryEncoding(static_cast<uint>(network_transport::runtime_info::tRegisterUIDs::TYPE), rrlib::serialization::tRegisterEntryEncoding::PUBLISH_REGISTER_ON_CHANGE);
        deserialization_info.SetRegisterEntryEncoding(static_cast<uint>(network_transport::runtime_info::tRegisterUIDs::TYPE), rrlib::serialization::tRegisterEntryEncoding::PUBLISH_REGISTER_ON_CHANGE);

        auto& unused_initialization_buffer = connection->peer.UnusedInitializationBuffer();

        if (java_partner)
        {
          serialization_info.SetRegisterEntryEncoding(static_cast<uint>(network_transport::runtime_info::tRegisterUIDs::CONVERSION_OPERATION), rrlib::serialization::tRegisterEntryEncoding::PUBLISH_REGISTER_ON_CHANGE);
          serialization_info.SetRegisterEntryEncoding(static_cast<uint>(network_transport::runtime_info::tRegisterUIDs::STATIC_CAST), rrlib::serialization::tRegisterEntryEncoding::PUBLISH_REGISTER_ON_CHANGE);
          serialization_info.SetRegisterEntryEncoding(static_cast<uint>(network_transport::runtime_info::tRegisterUIDs::SCHEME_HANDLER), rrlib::serialization::tRegisterEntryEncoding::PUBLISH_REGISTER_ON_CHANGE);
        }

        if (primary_connection && connection->SharedConnectionInfo().remote_runtime->GetPrimaryConnection() != connection)
        {
          connection->SharedConnectionInfo().remote_runtime = nullptr;
          connection->Close(); // we already have a connection of this type
          return;
        }
        else if (primary_connection)
        {
          connection->SharedConnectionInfo().input_stream_prototype.Reset(unused_initialization_buffer, deserialization_info);
          connection->SharedConnectionInfo().output_stream_prototype.Reset(unused_initialization_buffer, serialization_info);
        }
        else if (express_only_connection)
        {
          if (!connection->SharedConnectionInfo().remote_runtime->AddConnection(connection, false))
          {
            connection->SharedConnectionInfo().remote_runtime = nullptr;
            connection->Close(); // we already have a connection of this type
            return;
          }
        }

        connection->InitFrontBuffer();
        connection->peer.RunEventLoop();

        connection->SharedConnectionInfo().initial_reading_complete = true;
        if (connection->SharedConnectionInfo().initial_reading_complete && connection->SharedConnectionInfo().initial_writing_complete)
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
  tMessageBatchWriteHandler(std::shared_ptr<tConnection::tBase>& connection_base, const rrlib::serialization::tMemoryBuffer& buffer_to_send, bool& back_buffer_lock_variable) :
    connection(std::static_pointer_cast<tConnection>(connection_base)),
    buffer_to_send(&buffer_to_send),
    back_buffer_lock_variable(&back_buffer_lock_variable)
  {
    back_buffer_lock_variable = true;
    boost::asio::async_write(*(connection->socket), boost::asio::const_buffers_1(buffer_to_send.GetBufferPointer(0), buffer_to_send.GetSize()), *this);
  }

  void operator()(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error || bytes_transferred != buffer_to_send->GetSize())
    {
      connection->Close();
    }
    else
    {
      *back_buffer_lock_variable = false;
    }
  }

private:
  std::shared_ptr<tConnection> connection;
  const rrlib::serialization::tMemoryBuffer* buffer_to_send;
  bool* back_buffer_lock_variable;
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
      if (!connection->IsClosed())
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
    continue_at = connection->ProcessIncomingMessageBatch(rrlib::serialization::tFixedBuffer(connection->read_buffer.GetPointer(), connection->GetBytesToRead()), continue_at);

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
 */
class tStructureWriteHandler
{
public:

  tStructureWriteHandler(std::shared_ptr<tConnection> connection) :
    connection(connection),
    buffer(),
    ready_after_write(false)
  {
    if (connection->GetRemoteRuntime()->GetDesiredStructureInfo() == tStructureExchange::SHARED_PORTS)
    {
      buffer.reset(new rrlib::serialization::tMemoryBuffer(0));
      *buffer = SerializeSharedPorts(connection->peer.LocalRuntimeInfo()->SharedPortInfo(), connection->SharedConnectionInfo().output_stream_prototype);
      connection->SharedConnectionInfo().framework_elements_in_full_structure_exchange_sent_until_handle = std::numeric_limits<core::tFrameworkElement::tHandle>::max(); // we want to receive any updates now
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
      connection->SharedConnectionInfo().initial_structure_writing_complete = true;
      if (connection->SharedConnectionInfo().initial_structure_reading_complete)
      {
        connection->ready = true;
        connection->peer.SetPeerListChanged();
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
    assert(connection->GetRemoteRuntime()->GetDesiredStructureInfo() == tStructureExchange::FINSTRUCT ||
           connection->GetRemoteRuntime()->GetDesiredStructureInfo() == tStructureExchange::COMPLETE_STRUCTURE);
    rrlib::thread::tLock lock(core::tRuntimeEnvironment::GetInstance().GetStructureMutex(), false);
    if (lock.TryLock())
    {
      connection->peer.ProcessLocalRuntimeStructureChanges();  // to make sure we don't get any shared port events twice
      core::tFrameworkElement* framework_element_buffer[1000];
      size_t element_count = 0;
      if (connection->SharedConnectionInfo().framework_elements_in_full_structure_exchange_sent_until_handle <= std::numeric_limits<tHandle>::max())
      {
        element_count = core::tRuntimeEnvironment::GetInstance().GetAllElements(framework_element_buffer, 1000,
                        connection->SharedConnectionInfo().framework_elements_in_full_structure_exchange_sent_until_handle);
      }
      if (!buffer)
      {
        buffer.reset(new rrlib::serialization::tMemoryBuffer(element_count * 200 + 4));
      }
      rrlib::serialization::tOutputStream stream(*buffer, connection->SharedConnectionInfo().output_stream_prototype);
      stream.WriteInt(0); // placeholder for size
      for (size_t i = 0; i < element_count; i++)
      {
        bool relevant = framework_element_buffer[i]->IsReady() &&
                        ((connection->GetRemoteRuntime()->GetDesiredStructureInfo() == tStructureExchange::FINSTRUCT) ||
                         (!framework_element_buffer[i]->GetFlag(core::tFrameworkElement::tFlag::NETWORK_ELEMENT)));
        if (relevant)
        {
          network_transport::runtime_info::tLocalFrameworkElementInfo::Serialize(stream, *framework_element_buffer[i], connection->GetRemoteRuntime()->GetDesiredStructureInfo() == tStructureExchange::FINSTRUCT);
          FINROC_LOG_PRINT(DEBUG_VERBOSE_2, "Serializing ", *framework_element_buffer[i]);
        }
        connection->SharedConnectionInfo().framework_elements_in_full_structure_exchange_sent_until_handle = framework_element_buffer[i]->GetHandle() + 1;
      }
      stream.Close();
      size_t payload_size = buffer->GetSize() - 4;
      buffer->GetBuffer().PutInt(0, payload_size);
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
        connection->SharedConnectionInfo().initial_structure_reading_complete = true;
        if (connection->SharedConnectionInfo().initial_structure_writing_complete)
        {
          connection->ready = true;
          connection->peer.SetPeerListChanged();
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
    rrlib::serialization::tMemoryBuffer packet_buffer(read_buffer->GetPointer(), connection->bytes_to_read);
    rrlib::serialization::tInputStream stream(packet_buffer, connection->SharedConnectionInfo().input_stream_prototype);
    connection->GetRemoteRuntime()->ProcessStructurePacket(stream);

    // read size of next structure buffer
    connection->bytes_to_read = 0;
    boost::asio::async_read(*(connection->socket), boost::asio::mutable_buffers_1(connection->read_size_buffer.GetPointer(), 4), *this);
  }

private:

  std::shared_ptr<tConnection> connection;
  std::shared_ptr<rrlib::serialization::tFixedBuffer> read_buffer;
};


tConnection::tConnection(tPeerImplementation& peer, std::shared_ptr<boost::asio::ip::tcp::socket>& socket, int flags,
                         std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator, bool never_forget) :
  network_transport::generic_protocol::tConnection(*peer.LocalRuntimeInfo(), true),
  peer(peer),
  flags(flags),
  socket(socket),
  ready(false),
  read_size_buffer(4),
  read_buffer(cINITIAL_READ_BUFFER_SIZE),
  bytes_to_read(0),
  active_connect_indicator(active_connect_indicator),
  never_forget(never_forget),
  defer_read_timer(peer.IOService()),
  defer_write_timer(peer.IOService())
{
}

tConnection::~tConnection()
{}

void tConnection::CloseImplementation()
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
  if (ready)
  {
    peer.SetPeerListChanged();
  }
}

void tConnection::DoInitialStructureExchange(std::shared_ptr<tConnection>& connection)
{
  if (connection == connection->GetRemoteRuntime()->GetPrimaryConnection())
  {
    assert(this == connection.get());
    tStructureReadHandler structure_read_handler(connection);
    if (GetRemoteRuntime()->GetDesiredStructureInfo() != tStructureExchange::NONE)
    {
      tStructureWriteHandler structure_write_handler(connection);
    }
    else
    {
      SharedConnectionInfo().initial_structure_writing_complete = true;
    }
  }
  else
  {
    SharedConnectionInfo().initial_structure_reading_complete = true;
    SharedConnectionInfo().initial_structure_writing_complete = true;
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

void tConnection::ProcessPeerInfoMessage(rrlib::serialization::tMemoryBuffer& buffer)
{
  network_transport::generic_protocol::tPeerInfoMessage message;
  rrlib::serialization::tInputStream stream(buffer, SharedConnectionInfo().input_stream_prototype);
  message.Deserialize(stream, false);
  while (stream.ReadBoolean())
  {
    tPeerInfo peer(tPeerType::UNSPECIFIED);
    this->peer.DeserializePeerInfo(stream, peer);
    RRLIB_LOG_PRINT(DEBUG_VERBOSE_1, "Deserialized peer ", peer.ToString());
    this->peer.ProcessIncomingPeerInfo(peer);
  }
  message.FinishDeserialize(stream);
}

void tConnection::SendMessagePacket(std::shared_ptr<tBase>& self, const rrlib::serialization::tMemoryBuffer& buffer_to_send, bool& back_buffer_lock_variable)
{
  tMessageBatchWriteHandler write_handler(self, buffer_to_send, back_buffer_lock_variable);
}

void tConnection::TryToEstablishConnection(tPeerImplementation& peer, std::shared_ptr<boost::asio::ip::tcp::socket>& socket, int flags,
    std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator, bool never_forget)
{
  std::shared_ptr<tConnection> connection(new tConnection(peer, socket, flags, active_connect_indicator, never_forget));

  tInitialWriteHandler write_handler(connection);
  tInitialReadHandler read_handler(connection);
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
