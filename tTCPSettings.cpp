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
#include "tcp/tTCPSettings.h"
#include "core/datatype/tUnit.h"
#include "core/datatype/tBounds.h"
#include "core/datatype/tConstant.h"

namespace finroc
{
namespace tcp
{
tTCPSettings tTCPSettings::inst;
const int tTCPSettings::cCONNECTOR_THREAD_LOOP_INTERVAL;
const int tTCPSettings::cCONNECTOR_THREAD_SUBSCRIPTION_UPDATE_INTERVAL;
const int tTCPSettings::cMIN_PORTS_UPDATE_INTERVAL;
const int tTCPSettings::cDEQUEUE_QUEUE_SIZE;
const int tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS;
const int tTCPSettings::cAVG_PING_PACKETS;
const bool tTCPSettings::cDEBUG_TCP;
const int tTCPSettings::cDEBUG_TCP_NUMBER;
core::tIntSetting* tTCPSettings::max_not_acknowledged_packets_express = tTCPSettings::inst.Add("Maximum not acknowledged express packets", 4, true, &(core::tUnit::cNO_UNIT), core::tBounds(1, 40, true));
core::tIntSetting* tTCPSettings::max_not_acknowledged_packets_bulk = tTCPSettings::inst.Add("Maximum not acknowledged bulk packets", 2, true, &(core::tUnit::cNO_UNIT), core::tBounds(1, 40, true));
core::tIntSetting* tTCPSettings::min_update_interval_express = tTCPSettings::inst.Add("Minimum Express Update Interval", 25, true, &(core::tUnit::ms), core::tBounds(1, 2000, core::tConstant::cNO_MIN_TIME_LIMIT.get()));
core::tIntSetting* tTCPSettings::min_update_interval_bulk = tTCPSettings::inst.Add("Minimum Bulk Update Interval", 50, true, &(core::tUnit::ms), core::tBounds(1, 2000, core::tConstant::cNO_MIN_TIME_LIMIT.get()));
core::tIntSetting* tTCPSettings::critical_ping_threshold = tTCPSettings::inst.Add("Critical Ping Threshold", 1500, true, &(core::tUnit::ms), core::tBounds(50, 20000, core::tConstant::cNO_MAX_TIME_LIMIT.get()));
core::tBoolSetting* tTCPSettings::cDISPLAY_INCOMING_TCP_SERVER_COMMANDS = tTCPSettings::inst.Add("DISPLAY_INCOMING_TCP_SERVER_COMMANDS", true, true);
core::tBoolSetting* tTCPSettings::cDISPLAY_INCOMING_PORT_UPDATES = tTCPSettings::inst.Add("DISPLAY_INCOMING_TCP_SERVER_COMMANDS", false, true);

tTCPSettings::tTCPSettings() :
    core::tSettings("TCP Settings", "tcp", true)
{
}

} // namespace finroc
} // namespace tcp

