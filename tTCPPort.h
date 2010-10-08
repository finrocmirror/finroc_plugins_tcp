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

#ifndef PLUGINS__TCP__TTCPPORT_H
#define PLUGINS__TCP__TTCPPORT_H

#include "core/port/tPortCreationInfo.h"
#include "core/port/tAbstractPort.h"
#include "core/port/net/tNetPort.h"
#include "core/port/tPortFlags.h"

namespace finroc
{
namespace core
{
class tAbstractCall;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace tcp
{
class tTCPConnection;

/*!
 * \author Max Reichardt
 *
 * NetPort for TCP Connections
 */
class tTCPPort : public core::tNetPort
{
protected:

  /*!
   * Connection that TCP Port belongs to - has to be checked for null, before used -
   * is deleted deferred, so using it after checking (without waiting) is safe
   */
  tTCPConnection* connection;

  /*! Is port currently monitored? */
  bool monitored;

  /*! Update interval as requested by connection partner - -1 or smaller means no request */
  int16 update_interval_partner;

private:

  /*!
   * \return Publish data of this port over the network in forward direction when it changes?
   */
  inline bool PublishPortDataOverTheNetForward()
  {
    return GetPort()->IsInputPort() && GetPort()->GetStrategy() > 0;
  }

  /*!
   * \return Publish data of this port over the network in forward direction when it changes?
   */
  inline bool PublishPortDataOverTheNetReverse()
  {
    return GetPort()->IsOutputPort() && GetPort()->GetFlag(core::tPortFlags::cPUSH_STRATEGY_REVERSE);
  }

protected:

  /*!
   * Relevant for client ports - called whenever something changes that could have an impact on a server subscription
   */
  virtual void CheckSubscription()
  {
  }

  virtual void PortChanged();

  virtual void PrepareDelete();

  virtual void SendCall(core::tAbstractCall* mc);

  /*!
   * Set whether port is monitored for changes
   *
   * \param monitored2 desired state
   */
  void SetMonitored(bool monitored2);

public:

  /*!
   * \param pci Port Creation Info
   * \param connection Connection that TCP Port belongs to
   */
  tTCPPort(core::tPortCreationInfo pci, tTCPConnection* connection_);

  int16 GetUpdateIntervalForNet();

  virtual void PropagateStrategyFromTheNet(int16 strategy)
  {
    ::finroc::core::tNetPort::PropagateStrategyFromTheNet(strategy);
    SetMonitored(PublishPortDataOverTheNet());
  }

  //  @Override
  //  protected void sendCallReturn(AbstractCall mc) {
  //      TCPConnection c = connection;
  //      if (c != null) {
  //
  //          // okay, we received a pull return that needs to be forwarded over the net
  //          mc.setRemotePortHandle(mc.popCaller());
  //          c.sendCall(mc);
  //      } else {
  //
  //          // no connection - throw exception
  //          mc.setStatus(AbstractCall.CONNECTION_EXCEPTION);
  //          mc.returnToCaller();
  //      }
  //  }

  /*!
   * \return Publish data of this port over the network when it changes? (regardless of forward or reverse direction)
   */
  inline bool PublishPortDataOverTheNet()
  {
    return PublishPortDataOverTheNetForward() || PublishPortDataOverTheNetReverse();
  }

  virtual void SendCallReturn(core::tAbstractCall* mc);

};

} // namespace finroc
} // namespace tcp

#endif // PLUGINS__TCP__TTCPPORT_H
