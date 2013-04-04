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
/*!\file    plugins/tcp/tSettings.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 * \brief   Contains tSettings
 *
 * \b tSettings
 *
 * Various settings for TCP plugin.
 * Can be changed/tuned for better performance - some even at runtime via ports
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__tSettings_h__
#define __plugins__tcp__tSettings_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tFrameworkElement.h"
#include "plugins/parameters/tParameter.h"

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP plugin settings
/*!
 * Various settings for TCP plugin.
 * Can be changed/tuned for better performance - some even at runtime via ports
 */
class tSettings : public core::tFrameworkElement
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*! Loop Interval for Connector Thread... currently only 1ms since waiting is done depending on critical ping times instead */
  static constexpr rrlib::time::tDuration cCONNECTOR_THREAD_LOOP_INTERVAL = std::chrono::milliseconds(1);

  /*! How often will Connector Thread update subscriptions? */
  static constexpr rrlib::time::tDuration cCONNECTOR_THREAD_SUBSCRIPTION_UPDATE_INTERVAL = std::chrono::seconds(2);

  /*! Minimum Update Time for remote Ports */
  static constexpr rrlib::time::tDuration cMIN_PORTS_UPDATE_INTERVAL = std::chrono::milliseconds(200);

  /*! Size of dequeue queue in TCP Port */
  static const int cDEQUEUE_QUEUE_SIZE = 50;

  /*! Maximum not acknowledged Packet */
  static const int cMAX_NOT_ACKNOWLEDGED_PACKETS = 0x3F;
  // 64-1 (2^x for fast modulo)

  /*! Packets considered when calculating average ping time */
  static const int cAVG_PING_PACKETS = 0x7;
  // 8 (2^x for fast modulo)

  /*! Help for debugging: insert checks in data stream => more bandwidth */
  static const bool cDEBUG_TCP = true;

  /*! Help for debugging: this number will be inserted after every message when DEBUG_TCP is activated */
  static const uint8_t cDEBUG_TCP_NUMBER = 0xCD;

  /*! Maximum number of port to try to create a server port on - if default port is occupied */
  static const int cMAX_PORTS_TO_TRY_FOR_CREATING_SERVER_PORT = 100;

  /*! Maximum packets to send without acknowledgement in express connections */
  parameters::tParameter<uint32_t> max_not_acknowledged_packets_express;

  /*! Maximum packets to send without acknowledgement in bulk connections */
  parameters::tParameter<uint32_t> max_not_acknowledged_packets_bulk;

  /*! Minimum interval between sending data of the same port twice (express connection) */
  parameters::tParameter<rrlib::time::tDuration> min_update_interval_express;

  /*! Minimum interval between sending data of the same port twice (bulk connection) */
  parameters::tParameter<rrlib::time::tDuration> min_update_interval_bulk;

  /*! Critical ping threshold (if this is exceeded ports are notified of disconnect) */
  parameters::tParameter<rrlib::time::tDuration> critical_ping_threshold;


  /*! Singleton Instance */
  static tSettings& GetInstance();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Debug Settings */
  tSettings();

  ~tSettings();

  /*! Singleton Instance */
  static tSettings* instance;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
