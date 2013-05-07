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
/*!\file    plugins/tcp/test/pTestCollection.cpp
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

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/test/mTestCollection.h"

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
const char * const cPROGRAM_VERSION = "1.0";
const char * const cPROGRAM_DESCRIPTION = "This program performs various tests on connected network ports.";

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

finroc::tcp::test::mTestCollection* test_collection_module = NULL;

class tConnector : public tRuntimeListener
{
public:
  tConnector() : connected(false) {}

private:
  bool connected;

  virtual void OnFrameworkElementChange(tRuntimeListener::tEvent change_type, tFrameworkElement& element)
  {
    if ((!connected) && element.IsPort() && element.GetFlag(tFrameworkElement::tFlag::NETWORK_ELEMENT))
    {
      std::string peer_name = element.GetParent()->GetName();
      FINROC_LOG_PRINT(USER, "Found partner peer: ", peer_name, ". Connecting ports.");
      connected = true;

      // Connect all input ports to output ports with the same name
      for (auto it = test_collection_module->GetInputs().ChildPortsBegin(); it != test_collection_module->GetInputs().ChildPortsEnd(); ++it)
      {
        if (finroc::data_ports::IsDataFlowType(it->GetDataType()) || it->GetDataType() == finroc::tcp::test::cTYPE)
        {
          std::string source = "/TCP/" + peer_name + "/Main Thread/TestCollection/Output/" + it->GetName();
          it->ConnectTo(source);
          FINROC_LOG_PRINT(DEBUG, "Connecting to ", source);
        }
      }

      tAbstractPort* bb_port = static_cast<tAbstractPort*>(test_collection_module->GetOutputs().GetChild("float blackboard"));
      std::string source = "/TCP/" + peer_name + "/Main Thread/TestCollection/Input/" + bb_port->GetName();
      bb_port->ConnectTo(source);
      FINROC_LOG_PRINT(DEBUG, "Connecting to ", source);
    }
  }

  virtual void OnEdgeChange(tRuntimeListener::tEvent change_type, tAbstractPort& source, tAbstractPort& target)
  {
  }
};

tConnector connector;

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{
  links_are_unique = false;
}

//----------------------------------------------------------------------
// InitMainGroup
//----------------------------------------------------------------------
void InitMainGroup(finroc::structure::tThreadContainer *main_thread, std::vector<char*> remaining_args)
{
  test_collection_module = new finroc::tcp::test::mTestCollection(main_thread);
  tRuntimeEnvironment::GetInstance().AddListener(connector);

  main_thread->SetCycleTime(std::chrono::milliseconds(100));
}
