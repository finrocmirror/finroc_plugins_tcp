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
/*!\file    plugins/tcp/internal/tConnection.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 * \brief   Contains tConnection
 *
 * \b tConnection
 *
 * A single connection between two TCP sockets.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tConnection_h__
#define __plugins__tcp__internal__tConnection_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/asio/deadline_timer.hpp>
#include "plugins/network_transport/generic_protocol/tConnection.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tPeerInfo.h"

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
// Class declaration
//----------------------------------------------------------------------
//! TCP Connection
/*!
 * A single connection between two TCP sockets.
 */
class tConnection : public network_transport::generic_protocol::tConnection
{
  typedef network_transport::generic_protocol::tConnection tBase;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  ~tConnection();

  /*!
   * Tries to establish and initialize connection to remote runtime at provided open socket.
   * If this works, adds this connection to list of connected runtimes.
   *
   * \param peer Peer implementation that this connection belongs to
   * \param socket Connection's Network Socket
   * \param flags Flags for this connection (will be merged with flags that connection partner sends)
   * \param active_connect_indicator Active connect indicator, in case this is a client connection
   * \param never_forget Is this a connection to a peer that should never be forgotten?
   */
  static void TryToEstablishConnection(tPeerImplementation& peer, std::shared_ptr<boost::asio::ip::tcp::socket>& socket, int flags,
                                       std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator, bool never_forget = false);

  /*!
   * \return True when connection is ready for writing
   */
  bool IsReady() const
  {
    return ready;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class tInitialWriteHandler;
  friend class tInitialReadHandler;
  friend class tStructureWriteHandler;
  friend class tStructureReadHandler;
  friend class tMessageBatchReadHandler;
  friend class tMessageBatchWriteHandler;
  friend class tRemotePart;

  /*!
   * \param peer Peer implementation that this connection belongs to
   * \param socket Connection's Network Socket
   * \param flags Flags for this connection (will be merged with flags that connection partner sends)
   * \param active_connect_indicator Active connect indicator, in case this is a client connection
   * \param never_forget Is this a connection to a peer that should never be forgotten?
   */
  tConnection(tPeerImplementation& peer, std::shared_ptr<boost::asio::ip::tcp::socket>& socket, int flags,
              std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator, bool never_forget);


  /*! Peer implementation that this connection belongs to */
  tPeerImplementation& peer;

  /*! Flags for this connection */
  int flags;

  /*! Connection's Network Socket */
  std::shared_ptr<boost::asio::ip::tcp::socket> socket;

  /*! True when connection is ready for writing */
  bool ready;

  /*! Buffer to store size of next message batch to read in */
  rrlib::serialization::tFixedBuffer read_size_buffer;

  /*! Buffer to store next message batch to read in */
  rrlib::serialization::tFixedBuffer read_buffer;

  /*! Contains number of bytes to read in next message batch */
  size_t bytes_to_read;

  /*! Active connect indicator, in case this is a client connection */
  std::shared_ptr<tPeerInfo::tActiveConnect> active_connect_indicator;

  /*! Is this a connection to a peer that should never be forgotten? */
  bool never_forget;

  /*! Timer to defer reading (e.g. when structure lock cannot be acquired) */
  boost::asio::deadline_timer defer_read_timer;

  /*! Timer to defer writing (e.g. when structure lock cannot be acquired) */
  boost::asio::deadline_timer defer_write_timer;


  /*! Closes connection and removes it from remote part etc. */
  virtual void CloseImplementation() override;

  /*!
   * Possibly sends structure info.
   * After that, connection initialization is complete
   * and reader is started.
   */
  void DoInitialStructureExchange(std::shared_ptr<tConnection>& connection);

  /*!
   * Fills 'bytes_to_read' with value read from network stream and returns it
   */
  size_t GetBytesToRead();

  virtual void ProcessPeerInfoMessage(rrlib::serialization::tMemoryBuffer& buffer) override;

  virtual void SendMessagePacket(std::shared_ptr<tBase>& self, const rrlib::serialization::tMemoryBuffer& buffer_to_send, bool& back_buffer_lock_variable) override;

  /*! Contains information shared among primary and non-primary connections to a specific peer */
  tSharedConnectionInfo& SharedConnectionInfo()
  {
    return tBase::SharedConnectionInfo();
  }
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
