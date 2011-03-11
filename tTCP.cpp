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
#include "plugins/tcp/tTCP.h"
#include "rrlib/finroc_core_utils/tAutoDeleter.h"
#include "plugins/tcp/tTCPCommand.h"
#include "plugins/tcp/tTCPPeer.h"
#include "core/plugin/tPlugins.h"
#include "core/tFrameworkElement.h"

namespace finroc
{
namespace tcp
{
::std::shared_ptr<tTCP> tTCP::instance;
const int8 tTCP::cTCP_P2P_ID_EXPRESS, tTCP::cTCP_P2P_ID_BULK;
const int8 tTCP::cSET, tTCP::cSUBSCRIBE, tTCP::cUNSUBSCRIBE, tTCP::cCHANGE_EVENT, tTCP::cPING, tTCP::cPONG, tTCP::cPULLCALL, tTCP::cMETHODCALL, tTCP::cUPDATETIME, tTCP::cREQUEST_PORT_UPDATE, tTCP::cPORT_UPDATE, tTCP::cPULLCALL_RETURN, tTCP::cMETHODCALL_RETURN, tTCP::cPEER_INFO;
const int8 tTCP::cSUCCESS, tTCP::cFAIL;
util::tString tTCP::cDEFAULT_CONNECTION_NAME = "localhost:4444";
util::tReusablesPoolCR<tTCPCommand>* tTCP::tcp_commands = util::tAutoDeleter::AddStatic(new util::tReusablesPoolCR<tTCPCommand>());
tTCP::tCreateAction tTCP::creator1(tTCPPeer::cGUI_FILTER, "TCP", 0);
tTCP::tCreateAction tTCP::creator2(tTCPPeer::cDEFAULT_FILTER, "TCP ports only", 0);
tTCP::tCreateAction tTCP::creator3(tTCPPeer::cALL_AND_EDGE_FILTER, "TCP admin", core::tCreateExternalConnectionAction::cREMOTE_EDGE_INFO);

tTCP::tTCP()
{
  instance = ::std::shared_ptr<tTCP>(this);
}

tTCP::~tTCP()
{
  if (tcp_commands != NULL)
  {
    tcp_commands->ControlledDelete();
  }
}

tTCPCommand* tTCP::GetUnusedTCPCommand()
{
  tTCPCommand* tc = tcp_commands->GetUnused();
  if (tc == NULL)
  {
    tc = new tTCPCommand();
    tcp_commands->Attach(tc, false);
  }
  //tc.responsibleThread = ThreadUtil.getCurrentThreadId();
  return tc;
}

void tTCP::Init()
{
  //        Plugins.getInstance().registerExternalConnection(creator1);
  //        Plugins.getInstance().registerExternalConnection(creator2);
  //        Plugins.getInstance().registerExternalConnection(creator3);
}

tTCP::tCreateAction::tCreateAction(core::tFrameworkElementTreeFilter filter_, const util::tString& name_, int flags_) :
    filter(filter_),
    name(name_),
    flags(flags_),
    group()
{
  core::tPlugins::GetInstance()->RegisterExternalConnection(this);

  group = GetBinary((void*)Dummy);
}

core::tExternalConnection* tTCP::tCreateAction::CreateExternalConnection() const
{
  return new tTCPPeer(cDEFAULT_CONNECTION_NAME, filter);
}

core::tFrameworkElement* tTCP::tCreateAction::CreateModule(core::tFrameworkElement* parent, const util::tString& name_, core::tConstructorParameters* params) const
{
  core::tFrameworkElement* result = CreateExternalConnection();
  parent->AddChild(result);
  return result;
}

} // namespace finroc
} // namespace tcp

