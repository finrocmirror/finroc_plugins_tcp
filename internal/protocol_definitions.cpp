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
/*!\file    plugins/tcp/internal/protocol_definitions.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-05
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/protocol_definitions.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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
const char* cGREET_MESSAGE = "Greetings! I am a Finroc TCP peer.";

//----------------------------------------------------------------------
// Implementation
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

tOutputStream& operator << (tOutputStream& stream, const boost::asio::ip::address& address)
{
  stream.WriteBoolean(address.is_v6());
  if (address.is_v6())
  {
    boost::asio::ip::address_v6::bytes_type bytes = address.to_v6().to_bytes();
    stream.Write(bytes.begin(), bytes.size());
  }
  else
  {
    assert(address.is_v4());
    boost::asio::ip::address_v4::bytes_type bytes = address.to_v4().to_bytes();
    stream.Write(bytes.begin(), bytes.size());
  }
  return stream;
}

tInputStream& operator >> (tInputStream& stream, boost::asio::ip::address& address)
{
  bool v6 = stream.ReadBoolean();
  if (v6)
  {
    boost::asio::ip::address_v6::bytes_type bytes;
    stream.ReadFully(bytes.begin(), bytes.size());
    address = boost::asio::ip::address_v6(bytes);
  }
  else
  {
    boost::asio::ip::address_v4::bytes_type bytes;
    stream.ReadFully(bytes.begin(), bytes.size());
    address = boost::asio::ip::address_v4(bytes);
  }
  return stream;
}

}
}

