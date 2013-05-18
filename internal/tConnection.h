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
/*!\file    plugins/tcp/internal/tConnection.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 * \brief   Contains tConnection
 *
 * \b tConnection
 *
 * A single connection between two TCP sockets.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tConnection_h__
#define __plugins__tcp__internal__tConnection_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/asio/deadline_timer.hpp>
#include "rrlib/serialization/serialization.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/common/tRemoteTypes.h"
#include "plugins/tcp/internal/protocol_definitions.h"
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
class tNetworkPortInfo;
class tPeerImplementation;
class tRemotePart;
class tSerializedStructureChange;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP Connection
/*!
 * A single connection between two TCP sockets.
 */
class tConnection : private rrlib::util::tNoncopyable
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  ~tConnection();

  /*!
   * \return Write stream for current connection. Should only be accessed by TCP thread due to thread-safety issues.
   */
  rrlib::serialization::tOutputStream& CurrentWriteStream()
  {
    return current_write_stream;
  }

  /*!
   * Initializes connection
   * If this works, adds this connection to the respective tRemotePart
   *
   * \param peer Peer implementation that this connection belongs to
   * \param socket Connection's Network Socket
   * \param flags Flags for this connection (will be merged with flags that connection partner sends)
   * \param active_connect_indicator Active connect indicator, in case this is a client connection
   * \param never_forget Is this a connection to a peer that should never be forgotten?
   */
  static void InitConnection(tPeerImplementation& peer, std::shared_ptr<boost::asio::ip::tcp::socket>& socket, int flags,
                             std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator, bool never_forget = false);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class tInitialWriteHandler;
  friend class tInitialReadHandler;
  friend class tStructureWriteHandler;
  friend class tStructureReadHandler;
  friend class tMessageBatchReadHandler;
  friend class tMessageBatchWriteHandler;
  friend class tRemotePart;

  /*!
   * \param peer Peer implementation that this connection belongs to
   * \param socket Connection's Network Socket
   * \param flags Flags for this connection (will be merged with flags that connection partner sends)
   * \param active_connect_indicator Active connect indicator, in case this is a client connection
   * \param never_forget Is this a connection to a peer that should never be forgotten?
   */
  tConnection(tPeerImplementation& peer, std::shared_ptr<boost::asio::ip::tcp::socket>& socket, int flags,
              std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator, bool never_forget);


  /*! Peer implementation that this connection belongs to */
  tPeerImplementation& peer;

  /*! Flags for this connection */
  int flags;

  /*! Connection's Network Socket */
  std::shared_ptr<boost::asio::ip::tcp::socket> socket;

  /*! True after connection has been closed */
  bool closed;

  /*! Remote part that this connection belongs to */
  tRemotePart* remote_part;

  /*! Have initial reading and writing been completed? */
  bool initial_reading_complete, initial_writing_complete;

  /*! Have initial structure reading and writing been completed? */
  bool initial_structure_reading_complete, initial_structure_writing_complete;

  /*! True when connection is ready for writing */
  bool ready;

  /*! Buffer to store size of next message batch to read in */
  rrlib::serialization::tFixedBuffer read_size_buffer;

  /*! Buffer to store next message batch to read in */
  rrlib::serialization::tFixedBuffer read_buffer;

  /*! Contains number of bytes to read in next message batch */
  size_t bytes_to_read;

  /*! We use double-buffering for write buffers - they are swapped on every write operation */
  rrlib::serialization::tMemoryBuffer front_buffer, back_buffer;

  /*! True, while back buffer is written to stream. During this time, no further buffers can be written to stream */
  bool writing_back_buffer_to_stream;

  /*! Information on remote types */
  common::tRemoteTypes remote_types;

  /*! Current stream for writing data to (writes to front buffer) */
  rrlib::serialization::tOutputStream current_write_stream;

  /*! Active connect indicator, in case this is a client connection */
  std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator;

  /*! Is this a connection to a peer that should never be forgotten? */
  bool never_forget;

  /*! Timer to defer reading (e.g. when structure lock cannot be acquired) */
  boost::asio::deadline_timer defer_read_timer;

  /*! Timer to defer writing (e.g. when structure lock cannot be acquired) */
  boost::asio::deadline_timer defer_write_timer;

  /*!
   * Index of last acknowledgement request that was received and has to be sent
   * -1 if there are currently no pending acknowledgement requests
   */
  int16_t last_ack_request_index;

  /*! Index of last sent packet that was acknowledged by connection partner (0-0x7FFF)*/
  int16_t last_acknowledged_packet;

  /*! Index of next packet that is to be sent */
  int16_t next_packet_index;

  /*! Number of sent bulk packages */
  uint32_t sent_bulk_packets;

  /*! Data on sent packet: Time sent, bulk packets sent */
  typedef std::pair<rrlib::time::tTimestamp, uint32_t> tSentPacketData;

  /*! Data on packet n that was sent (Index is n % MAX_NOT_ACKNOWLEDGED_PACKETS => efficient and safe implementation (ring queue)) */
  std::array < tSentPacketData, tSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS + 1 > sent_packet_data;

  /*!
   * In case connection partner wants full or finstruct structure exchange:
   * Until which handle have structure elements been sent to connection partner?
   * (excludes element with handle of this variable)
   * (invalid as soon as initial_structure_writing_complete is true)
   */
  uint64_t framework_elements_in_full_structure_exchange_sent_until_handle;

  static_assert(sizeof(framework_elements_in_full_structure_exchange_sent_until_handle) > sizeof(core::tFrameworkElement::tHandle), "Must be larger");

  /*!
   * Contains buffer pools for RPC calls from this connection to specific local RPC ports.
   * Key is handle of local RPC port.
   */
  std::map<core::tFrameworkElement::tHandle, std::unique_ptr<data_ports::standard::tMultiTypePortBufferPool>> rpc_call_buffer_pools;


  /*! Closes connection and removes it from remote part etc. */
  void Close();

  /*!
   * Possibly sends structure info.
   * After that, connection initialization is complete
   * and reader is started.
   */
  void DoInitialStructureExchange(std::shared_ptr<tConnection> connection);

  /*!
   * Fills 'bytes_to_read' with value read from network stream and returns it
   */
  size_t GetBytesToRead();

  /*!
   * Processes message batch in read buffer
   *
   * \param start_at Start reading at specified offset. Is called with non-null, if processing has been deferred
   * \return Zero, if message batch was processed completely. Offset to continue processing at, if processing is deferred.
   */
  size_t ProcessMessageBatch(size_t start_at = 0);

  /*!
   * Called by TCP Thread whenever rpc ports were deleted
   *
   * \param deleted_ports List with handles of deleted RPC ports
   */
  void RpcPortsDeleted(std::vector<core::tFrameworkElement::tHandle>& deleted_ports);

  /*!
   * Send pending messages.
   * Swaps front buffer and back buffer. Reinitializes current write stream.
   */
  void SendPendingMessages(const rrlib::time::tTimestamp& time_now);

  /*!
   * Sends structure change to remote part
   *
   * \param structure_change Serialized structure change to send
   * \param structure_exchange_level Desired structure exchange level of partner
   */
  void SendStructureChange(const tSerializedStructureChange& structure_change, common::tStructureExchange structure_exchange_level);

  /*!
   * Send values of ports in list to network stream
   *
   * \return True if any data was written
   */
  bool SendPortData(std::vector<tNetworkPortInfo*>& port_list, const rrlib::time::tTimestamp& time_now);

  /*!
   * Serializes any new local data types that might have been added to rrlib::rtti::tType
   */
  void SerializeAnyNewTypes();
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
