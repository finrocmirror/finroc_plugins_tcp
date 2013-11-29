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
/*!\file    plugins/tcp/internal/tPlugin.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-11-29
 *
 * TCP transport plugin implementation
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tRuntimeEnvironment.h"
#include "plugins/network_transport/tNetworkTransportPlugin.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/tPeer.h"

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
namespace internal
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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP transport plugin implementation
/*!
 * TCP transport plugin implementation
 */
class tPlugin : public network_transport::tNetworkTransportPlugin
{
  virtual std::string Connect(core::tAbstractPort& local_port, const std::string& remote_runtime_uuid,
                              int remote_port_handle, const std::string remote_port_link)
  {
    core::tFrameworkElement* peer_element = core::tRuntimeEnvironment::GetInstance().GetChild("TCP");
    if (!peer_element)
    {
      return "No peer element instantiated";
    }
    return static_cast<tPeer*>(peer_element)->implementation->Connect(local_port, remote_runtime_uuid, remote_port_handle, remote_port_link);
  }

  virtual const char* GetId()
  {
    return "tcp";
  }

  virtual void Init()
  {
  }
};

static tPlugin cPLUGIN_INSTANCE;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
