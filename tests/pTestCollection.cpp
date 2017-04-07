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
/*!\file    plugins/tcp/tests/pTestCollection.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-20
 *
 * \b pTestCollection
 *
 * Performs various tests on connected network ports
 * (data ports, rpc ports and blackboards).
 * Two of these test programs need to be started on the same port
 * in order for the tests to start.
 *
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tRuntimeEnvironment.h"
#include <chrono>
//#include "plugins/development_utils/mQuickHacks.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/tests/mTestCollection.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::core;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program performs various tests on connected network ports.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = false;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{}

//----------------------------------------------------------------------
// InitMainGroup
//----------------------------------------------------------------------
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{
  finroc::structure::tTopLevelThreadContainer<> *main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);

  auto test_collection_module = new finroc::tcp::tests::mTestCollection(main_thread);

  // Connect all input ports to output ports with the same name
  for (auto it = test_collection_module->GetInputs().ChildPortsBegin(); it != test_collection_module->GetInputs().ChildPortsEnd(); ++it)
  {
    if (finroc::data_ports::IsDataFlowType(it->GetDataType()) || it->GetDataType() == finroc::tcp::tests::cTYPE)
    {
      rrlib::uri::tURI uri("tcp:/Main Thread/TestCollection/Output/" + it->GetName());
      it->ConnectTo(uri);
      FINROC_LOG_PRINT(DEBUG, "Connecting to ", uri.ToString());
    }
  }

  auto client_port = test_collection_module->float_blackboard_client.GetWritePort();
  auto server_port = test_collection_module->float_blackboard.GetWritePort();
  rrlib::uri::tURI uri("tcp:" + rrlib::uri::tURI(server_port.GetWrapped()->GetPath()).ToString());
  FINROC_LOG_PRINT(DEBUG, "Connecting to ", uri.ToString());
  client_port.ConnectTo(uri);


  //finroc::development_utils::mQuickHacks::PrintChildTree(finroc::core::tRuntimeEnvironment::GetInstance());

  main_thread->SetCycleTime(std::chrono::milliseconds(100));
}
