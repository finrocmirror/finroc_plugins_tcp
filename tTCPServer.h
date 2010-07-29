/**
 * You received this file as part of an advanced experimental
 * robotics framework prototype ('finroc')
 *
 * Copyright (C) 2007-2010 Max Reichardt,
 *   Robotics Research Lab, University of Kaiserslautern
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include "finroc_core_utils/tJCBase.h"

#ifndef PLUGINS__TCP__TTCPSERVER_H
#define PLUGINS__TCP__TTCPSERVER_H

#include "finroc_core_utils/net/tNetSocket.h"
#include "tcp/tTCP.h"
#include "finroc_core_utils/net/tTCPConnectionHandler.h"
#include "core/tFrameworkElement.h"
#include "finroc_core_utils/net/tTCPServer.h"

namespace finroc
{
namespace tcp
{
class tTCPPeer;

/*!
 * \author Max Reichardt
 *
 * TCP Server instance.
 *
 * Module to provide local ports to other robots using a P2P-TCP based
 * communication mechanism.
 */
class tTCPServer : public core::tFrameworkElement, public util::tTCPServer
{
private:

  /*! Port Server runs on */
  int port;

  /*! Try the following ports, if specified port is already occupied? */
  bool try_next_ports_if_occupied;

  /*! Is server ready and serving requests? */
  bool serving;

  /*! Peer that this server belongs to */
  tTCPPeer* peer;

public:

  /*! Log domain for this class */
  RRLIB_LOG_CREATE_NAMED_DOMAIN(log_domain, "tcp");

protected:

  virtual void PostChildInit();

  virtual void PrepareDelete()
  {
    util::tLock lock2(this);
    util::tTCPConnectionHandler::RemoveServer(this, port);
  }

public:

  /*!
   * \param port Port Server runs on
   * \param try_next_ports_if_occupied Try the following ports, if specified port is already occupied?
   * \param peer Peer that this server belongs to
   */
  tTCPServer(int port_, bool try_next_ports_if_occupied_, tTCPPeer* peer_);

  virtual void AcceptConnection(::std::tr1::shared_ptr<util::tNetSocket> s, int8 first_byte);

  virtual bool Accepts(int8 first_byte)
  {
    return (first_byte == tTCP::cTCP_P2P_ID_BULK) || (first_byte == tTCP::cTCP_P2P_ID_EXPRESS);
  }

  /*!
   * \return Port that server (finally) listens on
   */
  inline int GetPort()
  {
    assert(((IsInitialized())) && "Port is not fixed yet");
    return port;
  }

};

} // namespace finroc
} // namespace tcp

#endif // PLUGINS__TCP__TTCPSERVER_H
