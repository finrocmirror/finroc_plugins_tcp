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
#include "rrlib/util/patterns/singleton.h"
#include "core/plugin/tPlugins.h"
#include "core/tFrameworkElement.h"

#include "plugins/tcp/tTCP.h"
#include "plugins/tcp/tTCPPeer.h"

namespace finroc
{
namespace tcp
{
typedef rrlib::util::tSingletonHolder<tTCP, rrlib::util::singleton::Longevity> tTCPPlugin;
static inline unsigned int GetLongevity(tTCP*)
{
  return 0xDDDDDDDD; // should be deallocated before tAllocationRegister singleton
}

static int InitTCPPlugin()
{
  tTCPPlugin::Instance();
  return 0;
}

static int init_tcp_plugin = InitTCPPlugin();

const int8 tTCP::cTCP_P2P_ID_EXPRESS, tTCP::cTCP_P2P_ID_BULK;
const int8 tTCP::cSUCCESS, tTCP::cFAIL;
util::tString tTCP::cDEFAULT_CONNECTION_NAME = "localhost:4444";
tTCP::tCreateAction tTCP::creator1(tTCPPeer::cGUI_FILTER, "TCP", 0);
tTCP::tCreateAction tTCP::creator2(tTCPPeer::cDEFAULT_FILTER, "TCP ports only", 0);
tTCP::tCreateAction tTCP::creator3(tTCPPeer::cALL_AND_EDGE_FILTER, "TCP admin", core::tCreateExternalConnectionAction::cREMOTE_EDGE_INFO);

tTCP::tTCP()
{
  tcp_commands = new util::tReusablesPoolCR<tTCPCommand>();
}

tTCP::~tTCP()
{
  if (tcp_commands != NULL)
  {
    tcp_commands->ControlledDelete();
  }
}

tTCPCommand::tPtr tTCP::GetUnusedTCPCommand()
{
  tTCP& plugin = tTCPPlugin::Instance();
  tTCPCommand* tc = plugin.tcp_commands->GetUnused();
  if (tc == NULL)
  {
    tc = new tTCPCommand();
    plugin.tcp_commands->Attach(tc, false);
  }
  return tTCPCommand::tPtr(tc);
}

void tTCP::Init()
{
}

tTCP::tCreateAction::tCreateAction(core::tFrameworkElementTreeFilter filter_, const util::tString& name_, int flags_) :
  filter(filter_),
  name(name_),
  flags(flags_),
  group()
{
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

