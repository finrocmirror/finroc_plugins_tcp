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
/*!\file    plugins/tcp/tTCPPlugin.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2017-03-19
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/tTCPPlugin.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/getopt/parser.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tPeerImplementation.h"

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

namespace internal
{
class tTCPPluginInstance : public internal::tPeerImplementation
{};
}

namespace
{
internal::tTCPPluginInstance plugin_instance;

bool OptionsHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  // auto-connect mode
  rrlib::getopt::tOption auto_connect(name_to_option_map.at("tcp-auto-connect"));
  if (auto_connect->IsActive())
  {
    plugin_instance.par_auto_connect_to_all_peers.Set(rrlib::getopt::EvaluateValue(auto_connect) == "yes");
  }

  return true;
}
}

//----------------------------------------------------------------------
// tTCPPlugin constructors
//----------------------------------------------------------------------
tTCPPlugin::tTCPPlugin() :
  tNetworkTransportPlugin("tcp", "tcp"),
  par_connect_to(this, "Connect To"),
  par_preferred_server_port(this, "Preferred Server Port", 4444),
  par_try_next_ports_if_occupied(this, "Try Next Server Port If Occupied", true),
  par_auto_connect_to_all_peers(this, "Auto-connect To All Peers", true),
  par_server_listen_address(this, "Server Listen Address", "0.0.0.0"), // = "0.0.0.0";
  par_peer_type(this, "Peer Type", tPeerType::FULL),
  par_debug_tcp(this, "Debug TCP", true),
  par_max_ports_to_try_creating_server_port(this, "Max Ports To Try Creating Server Port", 100),
  par_min_update_interval_express(this, "Min Update Interval High Priority Data", std::chrono::milliseconds(0)),
  par_min_update_interval_bulk(this, "Min Update Interval Low Priority Data", std::chrono::milliseconds(40)),
  par_process_events_call_interval(this, "Process Events Call Interval", std::chrono::milliseconds(5)),
  par_process_low_priority_tasks_call_interval(this, "Process Low Priority Tasks Call Interval", std::chrono::milliseconds(500))
{
  rrlib::getopt::AddValue("tcp-auto-connect", 0, "Auto-connect to all peers that become known? (yes/no) - default is 'yes' (Note: this might change in future releases)", &OptionsHandler);
}

//----------------------------------------------------------------------
// tTCPPlugin destructor
//----------------------------------------------------------------------
tTCPPlugin::~tTCPPlugin()
{}

void tTCPPlugin::AddRuntimeToConnectTo(const std::string& address)
{
  auto list = *par_connect_to.GetPointer();
  list.push_back(address);
  par_connect_to.Set(list);
}

tTCPPlugin& tTCPPlugin::GetInstance()
{
  return plugin_instance;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
