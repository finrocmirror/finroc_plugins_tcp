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
/*!\file    plugins/tcp/internal/protocol_definitions.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-05
 *
 * Various definitions of the TCP protocol that peers use to communicate.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__protocol_definitions_h__
#define __plugins__tcp__internal__protocol_definitions_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/asio/ip/tcp.hpp>
#include "plugins/rpc_ports/definitions.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/common/tFrameworkElementInfo.h"
#include "plugins/tcp/internal/tUUID.h"

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

/*! When two peers connnect - they use this message to identify themselves as Finroc peers */
extern const char* cGREET_MESSAGE;

/*! TCP protocol version */
enum { cPROTOCOL_VERSION = 1 };

/*! Mode of peer */
enum tPeerType
{
  CLIENT_ONLY,  //!< Peer is client only
  SERVER_ONLY,  //!< Peer is server only
  FULL          //!< Peer is client and server
};

/*!
 * Protocol OpCodes
 */
enum class tOpCode : uint8_t
{
  // Opcodes for management connection
  SUBSCRIBE,         // Subscribe to data port
  UNSUBSCRIBE,       // Unsubscribe from data port
  PULLCALL,          // Pull call
  PULLCALL_RETURN,   // Returning pull call
  RPC_CALL,          // RPC call
  TYPE_UPDATE,       // Update on remote type info (typically desired update time)
  STRUCTURE_CREATE,  // Update on remote framework elements: Element created
  STRUCTURE_CHANGE,  // Update on remote framework elements: Port changed
  STRUCTURE_DELETE,  // Update on remote framework elements: Element deleted
  PEER_INFO,         // Information about other peers

  // Change event opcodes (from subscription - or for plain setting of port)
  PORT_VALUE_CHANGE,                         // normal variant
  SMALL_PORT_VALUE_CHANGE,                   // variant with max. 256 byte message length (3 bytes smaller than normal variant)
  SMALL_PORT_VALUE_CHANGE_WITHOUT_TIMESTAMP, // variant with max. 256 byte message length and no timestamp (11 bytes smaller than normal variant)

  // Used for messages without opcode
  OTHER
};

/*!
 * Flags for connection properties
 */
enum class tConnectionFlag : int
{
  MANAGEMENT_DATA = 0x1, //<! This connection is used to transfer management data (e.g. available ports, peer info, subscriptions, rpc calls)
  EXPRESS_DATA    = 0x2, //<! This connection transfers "express data" (port values of express ports) - candidate for UDP in the future
  BULK_DATA       = 0x4  //<! This connection transfers "bulk data" (port values of bulk ports) - candidate for UDP in the future
};

}
}
}

#include "plugins/tcp/internal/tMessage.h"

namespace finroc
{
namespace tcp
{
namespace internal
{

typedef rpc_ports::internal::tCallId tCallId;

typedef typename core::tFrameworkElement::tHandle tFrameworkElementHandle;

/*!
 * Helper to read message size from stream
 * Exists once for each opcode
 */
struct tMessageSizeReader
{
  tMessageSize message_size;
  size_t argument_size;

  size_t ReadMessageSize(rrlib::serialization::tInputStream& stream) const
  {
    if (message_size == tMessageSize::FIXED)
    {
      return argument_size;
    }
    else if (message_size == tMessageSize::VARIABLE_UP_TO_4GB)
    {
      return stream.ReadInt();
    }
    else
    {
      return stream.ReadNumber<uint8_t>();
    }
  }
};

/*!
 * Lookup for message size reader for each opcode
 */
extern const tMessageSizeReader message_size_for_opcodes[static_cast<int>(tOpCode::OTHER) + 1];

////////////////////
// Message types
////////////////////

// Parameters: [remote port handle][int16: strategy][bool: reverse push][int16: update interval][local port handle][encoding]
typedef tMessage < tOpCode::SUBSCRIBE, tMessageSize::FIXED, tFrameworkElementHandle, int16_t, bool,
        int16_t, tFrameworkElementHandle, rrlib::serialization::tDataEncoding > tSubscribeMessage;

// Parameters: [remote port handle]
typedef tMessage<tOpCode::UNSUBSCRIBE, tMessageSize::FIXED, tFrameworkElementHandle> tUnsubscribeMessage;

// Parameters: [int32: acknowledged message batch]
//typedef tMessage<tOpCode::ACK, tMessageSize::FIXED, int32_t> tAckMessage;

// Parameters: [remote port handle][call uid][desired encoding]
typedef tMessage<tOpCode::PULLCALL, tMessageSize::FIXED, tFrameworkElementHandle, tCallId, rrlib::serialization::tDataEncoding> tPullCall;

// Parameters: [call uid][failed?] after message: [type][timestamp][serialized data]
typedef tMessage<tOpCode::PULLCALL_RETURN, tMessageSize::VARIABLE_UP_TO_4GB, tCallId, bool> tPullCallReturn;

// Parameters: [remote port handle][call type] after message: [pass stream to DeserializeCallFunction of tMessage]
typedef tMessage<tOpCode::RPC_CALL, tMessageSize::VARIABLE_UP_TO_4GB, tFrameworkElementHandle, rpc_ports::tCallType> tRPCCall;

// Parameters: after message: [data type][int16: new update time]
typedef tMessage<tOpCode::TYPE_UPDATE, tMessageSize::VARIABLE_UP_TO_4GB> tTypeUpdateMessage;

// Parameters: [local port handle] after message: [tFrameworkElementInfo]
typedef tMessage<tOpCode::STRUCTURE_CREATE, tMessageSize::VARIABLE_UP_TO_4GB, tFrameworkElementHandle> tStructureCreateMessage;

// Parameters: [local port handle] after message: [tChangeablePortInfo and possibly edge info]
typedef tMessage<tOpCode::STRUCTURE_CHANGE, tMessageSize::VARIABLE_UP_TO_4GB, tFrameworkElementHandle> tStructureChangeMessage;

// Parameters: [local port handle]
typedef tMessage<tOpCode::STRUCTURE_DELETE, tMessageSize::FIXED, tFrameworkElementHandle> tStructureDeleteMessage;

// Parameters: TODO
typedef tMessage<tOpCode::PEER_INFO, tMessageSize::VARIABLE_UP_TO_4GB> tPeerInfoMessage;


// Parameters: [int32: remote port handle][encoding] after message: [binary blob or null-terminated string depending on type encoding]
typedef tMessage < tOpCode::PORT_VALUE_CHANGE, tMessageSize::VARIABLE_UP_TO_4GB, int32_t,
        rrlib::serialization::tDataEncoding > tPortValueChange;

// Parameters: [int32: remote port handle][encoding] after message: [binary blob or null-terminated string depending on type encoding]
typedef tMessage < tOpCode::SMALL_PORT_VALUE_CHANGE, tMessageSize::VARIABLE_UP_TO_255_BYTE, int32_t,
        rrlib::serialization::tDataEncoding > tSmallPortValueChange;

// Parameters: [int32: remote port handle][encoding] after message: [binary blob or null-terminated string depending on type encoding]
typedef tMessage < tOpCode::SMALL_PORT_VALUE_CHANGE_WITHOUT_TIMESTAMP, tMessageSize::VARIABLE_UP_TO_255_BYTE, int32_t,
        rrlib::serialization::tDataEncoding > tSmallPortValueChangeWithoutTimestamp;

// Initializes connection between two peers: [my UUID][peer type][peer name][structure exchange][connection flags][your address]
typedef tMessage < tOpCode::OTHER, tMessageSize::VARIABLE_UP_TO_4GB, tUUID, tPeerType, std::string, common::tStructureExchange,
        int32_t, boost::asio::ip::address > tConnectionInitMessage;

//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

namespace rrlib
{
namespace serialization
{

tOutputStream& operator << (tOutputStream& stream, const boost::asio::ip::address& address);
tInputStream& operator >> (tInputStream& stream, boost::asio::ip::address& address);

}
}


#endif
