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
/*!\file    plugins/tcp/internal/tServer.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tServer.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/asio/ip/v6_only.hpp>
#include "rrlib/thread/tThread.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tConnection.h"
#include "plugins/tcp/internal/tPeerImplementation.h"
#include "plugins/tcp/internal/util.h"

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

/*!
 * Handles accepted connections
 * We don't use boost bind to minimize overhead
 */
class tAcceptHandler
{
public:
  tAcceptHandler(tServer& implementation, std::shared_ptr<boost::asio::ip::tcp::acceptor>& acceptor) :
    implementation(&implementation),
    socket(new boost::asio::ip::tcp::socket(implementation.peer.IOService())),
    acceptor(acceptor)
  {}

  void operator()(const boost::system::error_code& error)
  {
    if (!error)
    {
      tConnection::TryToEstablishConnection(implementation->peer, socket, 0, nullptr);
    }
    else
    {
      FINROC_LOG_PRINT(ERROR, "Connection error ", error.category().name());
    }

    // Accept another connection
    implementation->StartConnectionAccept(acceptor);
  }

  tServer* implementation;

  /*! Boost asio server socket */
  std::shared_ptr<boost::asio::ip::tcp::socket> socket;

  /*! Acceptor pointer */
  std::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;
};

tServer::tServer(tPeerImplementation& peer) :
  tFrameworkElement(peer.GetPluginRootFrameworkElement(), "TCP Server", tFlag::NETWORK_ELEMENT),
  peer(peer)
{
}

tServer::~tServer() {}

void tServer::Run()
{
  assert(&rrlib::thread::tThread::CurrentThread() == peer.thread.get());

  // Setup server socket
  std::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;
  boost::system::error_code ec;

  boost::asio::ip::address listen_address = boost::asio::ip::address::from_string(peer.par_server_listen_address.Get());

  auto desired_port = peer.par_preferred_server_port.Get();
  bool try_next_ports_if_occupied = peer.par_try_next_ports_if_occupied.Get();
  for (int port_to_try = desired_port;
       port_to_try < (try_next_ports_if_occupied ? (desired_port + peer.par_max_ports_to_try_creating_server_port.Get()) : (desired_port + 1));
       port_to_try++)
  {
    try
    {
      boost::asio::ip::tcp::endpoint epoint(listen_address, port_to_try);
      acceptor.reset(new boost::asio::ip::tcp::acceptor(*peer.io_service));
      acceptor->open(epoint.protocol());
      acceptor->set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
      if (epoint.protocol() == boost::asio::ip::tcp::v6())
      {
        acceptor->set_option(boost::asio::ip::v6_only(false), ec);
        if (ec)
        {
          FINROC_LOG_PRINT(WARNING, "Could not enable additional IPv4 support");
          acceptor.reset();
          continue;
        }
      }
      acceptor->bind(epoint);
      acceptor->listen();
      peer.this_peer.uuid.port = port_to_try;
      break;
    }
    catch (std::exception& ex)
    {
      FINROC_LOG_PRINT(WARNING, "Could not listen on port: ", port_to_try, ".");
      acceptor.reset();
    }
  }
  if (!acceptor)
  {
    FINROC_LOG_PRINT(ERROR, "TCP server could not listen on any of the ", peer.par_max_ports_to_try_creating_server_port.Get(), " ports tried. TCP interface is not available.");
    return;
  }
  FINROC_LOG_PRINT(USER, "TCP server is listening on port ", peer.this_peer.uuid.port);

  // If no connect-address was specified and desired port was occupied - connect to part that is running there
  if (peer.par_auto_connect_to_all_peers.Get() && desired_port != peer.this_peer.uuid.port)
  {
    peer.AddRuntimeToConnectTo(std::string("localhost:") + std::to_string(desired_port));
  }

  // Accept connections on socket
  StartConnectionAccept(acceptor);
  peer.io_service->run();
}

void tServer::StartConnectionAccept(std::shared_ptr<boost::asio::ip::tcp::acceptor>& acceptor)
{
  internal::tAcceptHandler accept_handler(*this, acceptor);
  acceptor->async_accept(*accept_handler.socket, accept_handler);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
