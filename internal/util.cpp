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
#include <boost/algorithm/string.hpp>

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

std::vector<boost::asio::ip::tcp::endpoint> ParseAndResolveNetworkAddress(const std::string& network_address)
{
  std::vector<std::string> splitted;
  size_t colon_pos = network_address.rfind(':');

  if (colon_pos == std::string::npos)
  {
    throw std::runtime_error("Could not parse network address: " + network_address);
  }
  int port = atoi(network_address.substr(colon_pos + 1).c_str());
  std::string address_string = network_address.substr(0, colon_pos);
  if (port <= 20 || port > 65535)
  {
    throw std::runtime_error("Invalid port in network address: " + network_address);
  }
  try
  {
    std::vector<boost::asio::ip::tcp::endpoint> endpoints;
    for (const boost::asio::ip::address & addr : ResolveHostname(address_string))
    {
      endpoints.push_back(boost::asio::ip::tcp::endpoint(addr, port));
    }
    return endpoints;
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error("Could not resolve network address: " + network_address);
  }
}

std::vector<boost::asio::ip::address> ResolveHostname(const std::string hostname)
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
  std::vector<boost::asio::ip::address> addresses;
  while (endpoint_iterator != end)
  {
    addresses.push_back(endpoint_iterator->endpoint().address());
    endpoint_iterator++;
  }

  // sort, so that IPv6 is preferred
  std::sort(addresses.begin(), addresses.end(), [](const boost::asio::ip::address & a, const boost::asio::ip::address & b) -> bool { return a.is_v6(); });

  return addresses;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
