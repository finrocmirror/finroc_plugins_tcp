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
#include "tcp/tTCP.h"
#include "finroc_core_utils/tAutoDeleter.h"
#include "tcp/tTCPPeer.h"

namespace finroc
{
namespace tcp
{
::std::tr1::shared_ptr<tTCP> tTCP::instance;
const int8 tTCP::cTCP_P2P_ID_EXPRESS, tTCP::cTCP_P2P_ID_BULK;
const int8 tTCP::cSET, tTCP::cSUBSCRIBE, tTCP::cUNSUBSCRIBE, tTCP::cCHANGE_EVENT, tTCP::cPING, tTCP::cPONG, tTCP::cPULLCALL, tTCP::cMETHODCALL, tTCP::cUPDATETIME, tTCP::cREQUEST_PORT_UPDATE, tTCP::cPORT_UPDATE, tTCP::cPULLCALL_RETURN, tTCP::cMETHODCALL_RETURN, tTCP::cPEER_INFO;
const int8 tTCP::cSUCCESS, tTCP::cFAIL;
util::tString tTCP::cDEFAULT_CONNECTION_NAME = "localhost:4444";
util::tReusablesPoolCR<tTCPCommand>* tTCP::tcp_commands = util::tAutoDeleter::AddStatic(new util::tReusablesPoolCR<tTCPCommand>());

tTCP::tTCP() :
    creator2(this)
{
  instance = ::std::tr1::shared_ptr<tTCP>(this);
}

core::tExternalConnection* tTCP::CreateExternalConnection()
{
  //return new TCPClient(new FrameworkElementTreeFilter(CoreFlags.STATUS_FLAGS | CoreFlags.NETWORK_ELEMENT, CoreFlags.READY | CoreFlags.PUBLISHED));
  return new tTCPPeer(cDEFAULT_CONNECTION_NAME, tTCPPeer::cGUI_FILTER);
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

core::tExternalConnection* tTCP::tTCPLightweigtFlat::CreateExternalConnection()
{
  return new tTCPPeer(tTCP::cDEFAULT_CONNECTION_NAME, tTCPPeer::cDEFAULT_FILTER);
}

} // namespace finroc
} // namespace tcp

