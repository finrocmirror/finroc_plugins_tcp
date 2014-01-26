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
/*!\file    plugins/tcp/internal/tServer.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 * \brief   Contains tServer
 *
 * \b tServer
 *
 * TCP server implementation.
 * Opens TCP port and handles incoming connections.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tServer_h__
#define __plugins__tcp__internal__tServer_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/asio/ip/tcp.hpp>
#include "core/tFrameworkElement.h"

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

class tPeerImplementation;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP server implementation.
/*!
 * TCP server implementation.
 * Opens TCP port and handles incoming connections.
 */
class tServer : public core::tFrameworkElement
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \param peer Peer that this server belongs to
   */
  tServer(tPeerImplementation& peer);

  /*!
   * \return TCP Port Server is supposed to listen on
   */
  int DesiredPort()
  {
    return desired_port;
  }

  /*! Contains "main loop" of TCP Thread */
  void Run();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class tAcceptHandler;

  /*! Peer that this server belongs to */
  tPeerImplementation& peer;

  /*! TCP Port Server is supposed to listen on */
  const int desired_port;

  /*! Try the following ports, if specified port is already occupied? */
  const bool try_next_ports_if_occupied;

  /* The address that server is supposed to listen on ("::" will enable IPv6) */
  const std::string server_listen_address;


  ~tServer();

  /*! Initiates asynchronous accepting of another connection */
  void StartConnectionAccept(std::shared_ptr<boost::asio::ip::tcp::acceptor>& acceptor);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
