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
/*!\file    plugins/tcp/internal/protocol_definitions.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-05
 *
 * \brief
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
#include "plugins/network_transport/generic_protocol/definitions.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/tTCPPlugin.h"
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

/*!
 * Flags for connection properties
 */
enum class tConnectionFlag : uint32_t
{
  PRIMARY_CONNECTION = 0x1,  //<! This connection is used to transfer management data (e.g. available ports, peer info, subscriptions, rpc calls)
  EXPRESS_DATA    = 0x2,  //<! This connection transfers "express data" (port values of express ports) - candidate for UDP in the future
  BULK_DATA       = 0x4,  //<! This connection transfers "bulk data" (port values of bulk ports) - candidate for UDP in the future
  JAVA_PEER       = 0x8,  //<! Sent by Java Peers on connection initialization
  NO_DEBUG        = 0x10, //<! No debug information in protocol
};

// Initializes connection between two peers: [my UUID][peer type][peer name][structure exchange][connection flags][your address]
typedef network_transport::generic_protocol::tMessage < network_transport::generic_protocol::tOpCode, network_transport::generic_protocol::tOpCode::OTHER, network_transport::generic_protocol::tMessageSize::VARIABLE_UP_TO_4GB, tUUID, tPeerType, std::string, network_transport::runtime_info::tStructureExchange,
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
