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
/*!\file    plugins/tcp/common/tNetworkUpdateTimeSettings.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-13
 *
 * \brief   Contains tNetworkUpdateTimeSettings
 *
 * \b tNetworkUpdateTimeSettings
 *
 * Contains global settings for minimum network update intervals.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__common__tNetworkUpdateTimeSettings_h__
#define __plugins__tcp__common__tNetworkUpdateTimeSettings_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/parameters/tParameter.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/common/tUpdateTimeChangeListener.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace tcp
{
namespace common
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Network update time settings
/*!
 * Contains global settings for minimum network update intervals.
 */
class tNetworkUpdateTimeSettings : public core::tFrameworkElement
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*! Default minimum network update time (ms) */
  parameters::tParameter<int16_t> default_minimum_network_update_time;


  tNetworkUpdateTimeSettings();

  /*!
   * \param listener Listener to add
   */
  void AddListener(tUpdateTimeChangeListener& listener);

  /*!
   * \return Singleton instance
   */
  static tNetworkUpdateTimeSettings& GetInstance();

  /*!
   * \param type Data type
   * \return Minimum Network Update Interval for specified data type
   */
  static int16_t GetMinNetUpdateInterval(rrlib::rtti::tType& type);

  /*! Port Listener callback */
  void OnPortChange(const int16_t& value, data_ports::tChangeContext& change_context);

  /*!
   * \param listener Listener to remove
   */
  void RemoveListener(tUpdateTimeChangeListener& listener);

  /*!
   * \param type Data type
   * \param new_time New minimum Network Update Interval for specified data type
   */
  static void SetMinNetUpdateInterval(rrlib::rtti::tType& type, int16_t new_time);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! List with runtime listeners */
  rrlib::concurrent_containers::tSet < tUpdateTimeChangeListener*, rrlib::concurrent_containers::tAllowDuplicates::NO, rrlib::thread::tNoMutex,
        rrlib::concurrent_containers::set::storage::ArrayChunkBased<8, 31, definitions::cSINGLE_THREADED >> update_time_listeners;

  std::vector<parameters::tParameter<int16_t>> data_type_update_times;


  /*!
   * Fills data_type_update_times
   */
  void AddDataTypeParameters();

  /*!
   * Notify update time change listener of change
   *
   * \param dt Datatype whose default time has changed
   * \param time New time
   */
  inline void NotifyUpdateTimeChangeListener(rrlib::rtti::tType dt, int16_t time)
  {
    for (auto it = update_time_listeners.Begin(); it != update_time_listeners.End(); ++it)
    {
      (*it)->UpdateTimeChanged(dt, time);
    }
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
