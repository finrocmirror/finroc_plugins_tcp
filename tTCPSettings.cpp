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
#include "plugins/tcp/tTCPSettings.h"
#include "core/tRuntimeSettings.h"

namespace finroc
{
namespace tcp
{
tTCPSettings* tTCPSettings::inst = NULL;
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6))
constexpr rrlib::time::tDuration tTCPSettings::cCONNECTOR_THREAD_LOOP_INTERVAL;
constexpr rrlib::time::tDuration tTCPSettings::cCONNECTOR_THREAD_SUBSCRIPTION_UPDATE_INTERVAL;
constexpr rrlib::time::tDuration tTCPSettings::cMIN_PORTS_UPDATE_INTERVAL;
#else
rrlib::time::tDuration tTCPSettings::cCONNECTOR_THREAD_LOOP_INTERVAL = std::chrono::milliseconds(1);
rrlib::time::tDuration tTCPSettings::cCONNECTOR_THREAD_SUBSCRIPTION_UPDATE_INTERVAL = std::chrono::seconds(2);
rrlib::time::tDuration tTCPSettings::cMIN_PORTS_UPDATE_INTERVAL = std::chrono::milliseconds(200);
#endif
const int tTCPSettings::cDEQUEUE_QUEUE_SIZE;
const int tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS;
const int tTCPSettings::cAVG_PING_PACKETS;
const bool tTCPSettings::cDEBUG_TCP;
const int tTCPSettings::cDEBUG_TCP_NUMBER;

tTCPSettings::tTCPSettings() :
  core::tFrameworkElement(core::tRuntimeSettings::GetInstance(), "TCP"),
  max_not_acknowledged_packets_express("Maximum not acknowledged express packets", this, 4, core::tBounds<int>(1, 40, true)),
  max_not_acknowledged_packets_bulk("Maximum not acknowledged bulk packets", this, 2, core::tBounds<int>(1, 40, true)),
  min_update_interval_express("Minimum Express Update Interval", this, std::chrono::milliseconds(25), core::tBounds<rrlib::time::tDuration>(std::chrono::milliseconds(1), std::chrono::seconds(2))),
  min_update_interval_bulk("Minimum Bulk Update Interval", this, std::chrono::milliseconds(50), core::tBounds<rrlib::time::tDuration>(std::chrono::milliseconds(1), std::chrono::seconds(2))),
  critical_ping_threshold("Critical Ping Threshold", this, std::chrono::milliseconds(1500), core::tBounds<rrlib::time::tDuration>(std::chrono::milliseconds(50), std::chrono::seconds(20)))
{
}

tTCPSettings* tTCPSettings::GetInstance()
{
  if (inst == NULL)
  {
    inst = new tTCPSettings();
  }
  return inst;
}

} // namespace finroc
} // namespace tcp

