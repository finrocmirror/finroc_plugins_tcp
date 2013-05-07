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
 * Utility function.
 * Copies contents of specified memory buffer to newly allocated fixed buffer of exactly the contents' size
 *
 * \param memory_buffer Memory buffer whose data to copy
 * \return Fixed buffer with copied data
 */
inline rrlib::serialization::tFixedBuffer CopyToNewFixedBuffer(rrlib::serialization::tMemoryBuffer& memory_buffer)
{
  rrlib::serialization::tFixedBuffer fixed_buffer(memory_buffer.GetSize());
  memcpy(fixed_buffer.GetPointer(), memory_buffer.GetBufferPointer(0), fixed_buffer.Capacity());
  return fixed_buffer;
}

/*!
 * Parses and resolves network address in specified string
 * Throws exception address cannot be parsed or resolved
 *
 * \param network_address Network address (e.g. 'localhost:4444')
 * \return TCP endpoint corresponding to this address
 */
boost::asio::ip::tcp::endpoint ParseAndResolveNetworkAddress(const std::string& network_address);

/*!
 * Look up an IP address for the specified host name
 * Throws exception if host could not be found
 * Prefers IPv6 addresses
 *
 * \return Returns an IP address
 */
boost::asio::ip::address ResolveHostname(const std::string host_name);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
