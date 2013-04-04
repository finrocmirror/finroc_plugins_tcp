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
/*!\file    plugins/tcp/internal/util.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-02-21
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/util.h"

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

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

boost::asio::ip::tcp::endpoint ParseAndResolveNetworkAddress(const std::string& network_address)
{
  std::vector<std::string> splitted;
  boost::split(splitted, network_address, boost::is_any_of(":"));
  if (splitted.size() != 2)
  {
    throw std::runtime_error("Could not parse network address: " + network_address);
  }
  int port = atoi(splitted[1].c_str());
  if (port <= 20 || port > 65535)
  {
    throw std::runtime_error("Invalid port in network address: " + network_address);
  }
  try
  {
    boost::asio::ip::address address = ResolveHostname(splitted[0]);
    return boost::asio::ip::tcp::endpoint(address, port);
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error("Could not resolve network address: " + network_address);
  }
}

boost::asio::ip::address ResolveHostname(const std::string hostname)
{
  boost::asio::io_service io;
  boost::asio::ip::tcp::resolver resolver(io);
  boost::asio::ip::tcp::resolver::query query(hostname, "");
  boost::system::error_code ec;
  boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query, ec);

  if (ec)
  {
    throw std::runtime_error("Could not resolve the hostname " + hostname);
  }

  boost::asio::ip::tcp::resolver::iterator end;
  boost::asio::ip::address v4_address;
  bool first = true;
  while (endpoint_iterator != end)
  {
    if (endpoint_iterator->endpoint().protocol() == boost::asio::ip::tcp::v6())
    {
      return endpoint_iterator->endpoint().address();
    }
    else if (first)
    {
      v4_address = endpoint_iterator->endpoint().address();
    }
    endpoint_iterator++;
    first = false;
  }
  return v4_address;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
