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
#include "rrlib/finroc_core_utils/tJCBase.h"

#ifndef PLUGINS__TCP__TTCPSETTINGS_H
#define PLUGINS__TCP__TTCPSETTINGS_H

#include "core/parameter/tParameterNumeric.h"
#include "core/datatype/tUnit.h"
#include "core/datatype/tBounds.h"
#include "core/datatype/tConstant.h"
#include "core/tFrameworkElement.h"

namespace finroc
{
namespace tcp
{
/*!
 * \author Max Reichardt
 *
 * TCP Settings
 */
class tTCPSettings : public core::tFrameworkElement
{
private:

  /*! Singleton Instance */
  static tTCPSettings* inst;

public:

  /*! Loop Interval for Connector Thread... currently only 1ms since waiting is done depending on critical ping times instead */
  static const int cCONNECTOR_THREAD_LOOP_INTERVAL = 1;

  /*! How often will Connector Thread update subscriptions? (in ms) */
  static const int cCONNECTOR_THREAD_SUBSCRIPTION_UPDATE_INTERVAL = 2000;

  /*! Minimum Update Time for remote Ports */
  static const int cMIN_PORTS_UPDATE_INTERVAL = 200;

  /*! Size of dequeue queue in TCP Port */
  static const int cDEQUEUE_QUEUE_SIZE = 50;

  /*! Maximum not acknowledged Packet */
  static const int cMAX_NOT_ACKNOWLEDGED_PACKETS = 0x1F;
  // 32 (2^x for fast modulo)

  /*! Packets considered when calculating avergage ping time */
  static const int cAVG_PING_PACKETS = 0x7;
  // 8 (2^x for fast modulo)

  /*! Help for debugging: insert checks in data stream => more bandwidth */
  static const bool cDEBUG_TCP = false;

  /*! Help for debugging: this number will be inserted after every command when DEBUG_TCP is activated */
  static const int cDEBUG_TCP_NUMBER = 0xCAFEBABE;

  // Port settings
  core::tParameterNumeric<int> max_not_acknowledged_packets_express;

  core::tParameterNumeric<int> max_not_acknowledged_packets_bulk;

  core::tParameterNumeric<int> min_update_interval_express;

  core::tParameterNumeric<int> min_update_interval_bulk;

  core::tParameterNumeric<int> critical_ping_threshold;

private:

  /*! Debug Settings */
  tTCPSettings();

public:

  static tTCPSettings* GetInstance();

  inline static void InitInstance()
  {
    GetInstance()->Init();
  }

};

} // namespace finroc
} // namespace tcp

#endif // PLUGINS__TCP__TTCPSETTINGS_H
