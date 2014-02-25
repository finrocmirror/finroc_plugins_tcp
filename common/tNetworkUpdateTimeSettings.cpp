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
/*!\file    plugins/tcp/common/tNetworkUpdateTimeSettings.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-13
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/common/tNetworkUpdateTimeSettings.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/rtti/tTypeAnnotation.h"
#include "core/tPlugin.h"
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
namespace common
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

namespace internal
{

/*!
 * \author Max Reichardt
 *
 * RPC mechanism plugin
 */
class tNetworkBasePlugin : public core::tPlugin
{

  virtual void Init() override
  {
    tNetworkUpdateTimeSettings::GetInstance();
  }
};

static tNetworkBasePlugin rpc_plugin_instance;

class tUpdateTimeStorage : public rrlib::rtti::tTypeAnnotation
{
public:
  int16_t update_time;
};

static rrlib::thread::tMutex listener_mutex;

}


tNetworkUpdateTimeSettings::tNetworkUpdateTimeSettings() :
  tFrameworkElement(&core::tRuntimeSettings::GetInstance(), "Default minimum network update times"),
  default_minimum_network_update_time("Global Default", this, 40, data_ports::tBounds<int16_t>(1, 2000)),
  update_time_listeners(),
  data_type_update_times()
{
  default_minimum_network_update_time.AddListener(*this);
  AddDataTypeParameters();
}

void tNetworkUpdateTimeSettings::AddDataTypeParameters()
{
  while (data_type_update_times.size() < rrlib::rtti::tType::GetTypeCount())
  {
    rrlib::rtti::tType dt = rrlib::rtti::tType::GetType(data_type_update_times.size());
    if (dt != NULL && data_ports::IsDataFlowType(dt))
    {
      parameters::tParameter<int16_t> p(dt.GetName(), this, -1, data_ports::tBounds<int16_t>(-1, 10000));
      data_type_update_times.push_back(p);
      p.AddListener(*this);
    }
    else
    {
      data_type_update_times.emplace_back();
    }
  }
}

void tNetworkUpdateTimeSettings::AddListener(tUpdateTimeChangeListener& listener)
{
  rrlib::thread::tLock lock(internal::listener_mutex);
  update_time_listeners.Add(&listener);
}

tNetworkUpdateTimeSettings& tNetworkUpdateTimeSettings::GetInstance()
{
  static tNetworkUpdateTimeSettings* settings = new tNetworkUpdateTimeSettings();
  return *settings;
}

int16_t tNetworkUpdateTimeSettings::GetMinNetUpdateInterval(rrlib::rtti::tType& type)
{
  internal::tUpdateTimeStorage* update_time = type.GetAnnotation<internal::tUpdateTimeStorage>();
  return update_time ? update_time->update_time : -1;
}

void tNetworkUpdateTimeSettings::OnPortChange(const int16_t& value, data_ports::tChangeContext& change_context)
{
  if (&change_context.Origin() == default_minimum_network_update_time.GetWrapped())
  {
    for (auto it = update_time_listeners.Begin(); it != update_time_listeners.End(); ++it)
    {
      (*it)->UpdateTimeChanged(rrlib::rtti::tType(), value);
    }
  }
  else
  {
    rrlib::rtti::tType dt = rrlib::rtti::tType::FindType(change_context.Origin().GetName());
    for (auto it = update_time_listeners.Begin(); it != update_time_listeners.End(); ++it)
    {
      (*it)->UpdateTimeChanged(dt, value);
    }
  }
}

void tNetworkUpdateTimeSettings::RemoveListener(tUpdateTimeChangeListener& listener)
{
  rrlib::thread::tLock lock(internal::listener_mutex);
  update_time_listeners.Remove(&listener);
}

void tNetworkUpdateTimeSettings::SetMinNetUpdateInterval(rrlib::rtti::tType& type, int16_t new_time)
{
  tNetworkUpdateTimeSettings& instance = GetInstance();
  instance.AddDataTypeParameters();
  internal::tUpdateTimeStorage* update_time = type.GetAnnotation<internal::tUpdateTimeStorage>();
  if (!update_time)
  {
    update_time = new internal::tUpdateTimeStorage();
    type.AddAnnotation(update_time);
  }
  if (update_time->update_time != new_time)
  {
    update_time->update_time = new_time;
    instance.data_type_update_times[type.GetUid()].Set(new_time);
    //NotifyUpdateTimeChangeListener(type, new_time); should happen automatically
  }
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
