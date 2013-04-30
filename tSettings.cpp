//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/tcp/tSettings.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/tSettings.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tRuntimeSettings.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

tSettings* tSettings::instance = NULL;
constexpr rrlib::time::tDuration tSettings::cCONNECTOR_THREAD_LOOP_INTERVAL;
constexpr rrlib::time::tDuration tSettings::cCONNECTOR_THREAD_SUBSCRIPTION_UPDATE_INTERVAL;
constexpr rrlib::time::tDuration tSettings::cMIN_PORTS_UPDATE_INTERVAL;
const int tSettings::cDEQUEUE_QUEUE_SIZE;
const int tSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS;
const int tSettings::cAVG_PING_PACKETS;
const bool tSettings::cDEBUG_TCP;
const uint8_t tSettings::cDEBUG_TCP_NUMBER;
const int tSettings::cMAX_PORTS_TO_TRY_FOR_CREATING_SERVER_PORT;


tSettings::tSettings() :
  core::tFrameworkElement(&core::tRuntimeSettings::GetInstance(), "TCP"),
  max_not_acknowledged_packets_express("Maximum not acknowledged express packets", this, 4, data_ports::tBounds<uint32_t>(1, 40, true)),
  max_not_acknowledged_packets_bulk("Maximum not acknowledged bulk packets", this, 2, data_ports::tBounds<uint32_t>(1, 40, true)),
  min_update_interval_express("Minimum Express Update Interval", this, std::chrono::milliseconds(25), data_ports::tBounds<rrlib::time::tDuration>(std::chrono::milliseconds(1), std::chrono::seconds(2))),
  min_update_interval_bulk("Minimum Bulk Update Interval", this, std::chrono::milliseconds(50), data_ports::tBounds<rrlib::time::tDuration>(std::chrono::milliseconds(1), std::chrono::seconds(2))),
  critical_ping_threshold("Critical Ping Threshold", this, std::chrono::milliseconds(1500), data_ports::tBounds<rrlib::time::tDuration>(std::chrono::milliseconds(50), std::chrono::seconds(20)))
{
  assert(max_not_acknowledged_packets_bulk.Get() == 2);
}

tSettings::~tSettings()
{}

tSettings& tSettings::GetInstance()
{
  if (instance == NULL)
  {
    instance = new tSettings();
    instance->Init();
  }
  return *instance;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
