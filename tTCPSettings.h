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

#ifndef PLUGINS__TCP__TTCPSETTINGS_H
#define PLUGINS__TCP__TTCPSETTINGS_H

#include "core/settings/tSetting.h"
#include "core/tRuntimeSettings.h"
#include "core/settings/tSettings.h"

namespace finroc
{
namespace tcp
{
/*!
 * \author Max Reichardt
 *
 * TCP Settings
 */
class tTCPSettings : public core::tSettings
{
private:

  /*! Singleton Instance */
  static tTCPSettings inst;

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

  //  /** TCP Settings from tcp.conf */
  //  private static final Properties defaultSettings;

  //  static {
  //      // init settings
  //      defaultSettings = new Properties();
  //      try {
  //          defaultSettings.load(TCP.instance.getConfFile());
  //      } catch (Exception e) {
  //          e.printStackTrace();
  //      }
  //  }

  // Port settings
  static core::tIntSetting* max_not_acknowledged_packets_express;

  static core::tIntSetting* max_not_acknowledged_packets_bulk;

  static core::tIntSetting* min_update_interval_express;

  static core::tIntSetting* min_update_interval_bulk;

  static core::tIntSetting* critical_ping_threshold;

  /*! Debug Settings */
  static core::tBoolSetting* cDISPLAY_INCOMING_TCP_SERVER_COMMANDS;

  static core::tBoolSetting* cDISPLAY_INCOMING_PORT_UPDATES;

private:

  tTCPSettings();

public:

  inline static void InitInstance()
  {
    inst.Init(core::tRuntimeSettings::GetInstance());
  }

};

} // namespace finroc
} // namespace tcp

#endif // PLUGINS__TCP__TTCPSETTINGS_H
