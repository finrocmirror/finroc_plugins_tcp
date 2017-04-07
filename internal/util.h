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
/*!\file    plugins/tcp/internal/util.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-02-21
 *
 * \brief
 *
 * Contains various utility functions for TCP plugin implementation
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__util_h__
#define __plugins__tcp__internal__util_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/asio/ip/tcp.hpp>
#include "rrlib/serialization/serialization.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/protocol_definitions.h"

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
// Function declarations
//----------------------------------------------------------------------


/*!
 * Parses and resolves network address in specified string
 * Throws exception if address cannot be parsed or resolved
 *
 * \param network_address Network address (e.g. 'localhost:4444')
 * \return std::vector of TCP endpoints corresponding to this address
 */
std::vector<boost::asio::ip::tcp::endpoint> ParseAndResolveNetworkAddress(const std::string& network_address);

/*!
 * Look up the IP addresses for the specified host name
 * Throws exception if host could not be found
 * Prefers IPv6 addresses (they are stored at the beginning of the vector)
 *
 * \return std::vector of IP addresses
 */
std::vector<boost::asio::ip::address> ResolveHostname(const std::string host_name);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
