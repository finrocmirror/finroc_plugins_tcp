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

#ifndef PLUGINS__TCP__TPEERLIST_H
#define PLUGINS__TCP__TPEERLIST_H

#include "finroc_core_utils/container/tSimpleList.h"
#include "finroc_core_utils/net/tIPSocketAddress.h"
#include "finroc_core_utils/stream/tInputStreamBuffer.h"
#include "finroc_core_utils/net/tIPAddress.h"
#include "finroc_core_utils/stream/tOutputStreamBuffer.h"
#include "core/port/net/tAbstractPeerTracker.h"

namespace finroc
{
namespace tcp
{
/*!
 * \author Max Reichardt
 *
 * List of network peers that can be connected to.
 * Depends on external inputs.
 * Checks for duplicates etc.
 *
 * TODO: Implement proper PeerTracker based on libavahi
 */
class tPeerList : public core::tAbstractPeerTracker
{
private:

  /*! List of peers */
  util::tSimpleList<util::tIPSocketAddress> peers;

  /*! Current version of list */
  volatile int revision;

  /*! Server port of own peer */
  int server_port;

public:

  /*! Log domain for this class */
  RRLIB_LOG_CREATE_NAMED_DOMAIN(log_domain, "tcp");

  /*! @param serverPort Server port of own peer */
  tPeerList(int server_port_, int lock_order);

  //  /**
  //   * \param port Port that we are listening on (used so that we don't connect to ourselves)
  //   */
  //  public PeerList(int port) {
  //      addPeer(IPSocketAddress.createUnresolved("localhost", port));
  //      addPeer(I)
  //  }

  void AddPeer(util::tIPSocketAddress isa, bool notify_on_change);

  /*!
   * Deserialize addresses - and complete our own list
   *
   * \param ci Input Stream
   * \param own_address Our own address from remote view
   * \param partner_address IP address of partner
   */
  void DeserializeAddresses(util::tInputStreamBuffer* ci, util::tIPAddress own_address, util::tIPAddress partner_address);

  /*!
   * \return Revision of peer list (incremented with each change)
   */
  inline int GetRevision()
  {
    return revision;
  }

  void RemovePeer(util::tIPSocketAddress isa);

  /*!
   * Serialize all known addresses
   *
   * \param co Output Stream
   */
  void SerializeAddresses(util::tOutputStreamBuffer* co);

};

} // namespace finroc
} // namespace tcp

#endif // PLUGINS__TCP__TPEERLIST_H
