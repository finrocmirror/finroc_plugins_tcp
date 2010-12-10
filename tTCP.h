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
#include "rrlib/finroc_core_utils/tJCBase.h"

#ifndef PLUGINS__TCP__TTCP_H
#define PLUGINS__TCP__TTCP_H

#include "rrlib/finroc_core_utils/container/tReusablesPoolCR.h"
#include "core/plugin/tPlugin.h"
#include "core/tFrameworkElementTreeFilter.h"
#include "core/plugin/tCreateExternalConnectionAction.h"

namespace finroc
{
namespace core
{
class tPluginManager;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace tcp
{
class tTCPPeer;
class tTCPCommand;

/*!
 * \author Max Reichardt
 *
 * Plugin for P2P TCP connections
 */
class tTCP : public util::tUncopyableObject, public core::tPlugin
{
  /*!
   * Class for TCP create-Actions
   */
  class tCreateAction : public util::tObject, public core::tCreateExternalConnectionAction
  {
  private:

    /*! Filter to used for this connection type */
    core::tFrameworkElementTreeFilter filter;

    /*! Name of connection type */
    util::tString name;

    /*! Flags to use */
    int flags;

    /*! Name of module type */
    util::tString group;

  public:

    tCreateAction(core::tFrameworkElementTreeFilter filter_, const util::tString& name_, int flags_);

    virtual core::tExternalConnection* CreateExternalConnection() const;

    static void Dummy() {}

    virtual core::tFrameworkElement* CreateModule(core::tFrameworkElement* parent, const util::tString& name_, core::tConstructorParameters* params) const;

    virtual int GetFlags() const
    {
      return flags;
    }

    virtual util::tString GetModuleGroup() const
    {
      return group;
    }

    virtual util::tString GetName() const
    {
      return name;
    }

    virtual const core::tStructureParameterList* GetParameterTypes() const
    {
      return NULL;
    }

  };

private:

  /*! Pool with Reusable TCP Commands (SUBSCRIBE & UNSUBSCRIBE) */
  static util::tReusablesPoolCR<tTCPCommand>* tcp_commands;

public:

  /*! Singleton instance of TCP plugin */
  static ::std::tr1::shared_ptr<tTCP> instance;

  /*! Stream IDs for different connection types */
  static const int8 cTCP_P2P_ID_EXPRESS = 9, cTCP_P2P_ID_BULK = 10;

  /*! Protocol OpCodes */
  static const int8 cSET = 1, cSUBSCRIBE = 2, cUNSUBSCRIBE = 3, cCHANGE_EVENT = 4, cPING = 5, cPONG = 6, cPULLCALL = 7, cMETHODCALL = 8, cUPDATETIME = 9, cREQUEST_PORT_UPDATE = 10, cPORT_UPDATE = 11, cPULLCALL_RETURN = 12, cMETHODCALL_RETURN = 13, cPEER_INFO = 14;

  /*! Return Status */
  static const int8 cSUCCESS = 100, cFAIL = 101;

  /*! Default network name */
  static util::tString cDEFAULT_CONNECTION_NAME;

  /*! Standard TCP connection creator */
  static tCreateAction creator1;

  /*! Alternative TCP connection creator */
  static tCreateAction creator2;

  /*! Complete TCP connection creator */
  static tCreateAction creator3;

  tTCP();

  virtual ~tTCP();

  /*!
   * \return Unused TCP Command
   */
  static tTCPCommand* GetUnusedTCPCommand();

  virtual void Init(core::tPluginManager& mgr);

};

} // namespace finroc
} // namespace tcp

#endif // PLUGINS__TCP__TTCP_H
