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

#ifndef plugins__tcp__tTCP_h__
#define plugins__tcp__tTCP_h__

#include "rrlib/finroc_core_utils/definitions.h"
#include "rrlib/finroc_core_utils/container/tReusablesPoolCR.h"

#include "core/plugin/tPlugin.h"
#include "core/tFrameworkElementTreeFilter.h"
#include "core/plugin/tCreateExternalConnectionAction.h"

#include "plugins/tcp/tTCPCommand.h"

namespace finroc
{
namespace tcp
{
class tTCPPeer;

/*!
 * \author Max Reichardt
 *
 * Plugin for P2P TCP connections
 */
class tTCP : public core::tPlugin
{
  /*!
   * Class for TCP create-Actions
   */
  class tCreateAction : public core::tCreateExternalConnectionAction
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

    virtual const core::tStaticParameterList* GetParameterTypes() const
    {
      return NULL;
    }

  };

private:

  /*! Pool with Reusable TCP Commands (SUBSCRIBE & UNSUBSCRIBE) */
  util::tReusablesPoolCR<tTCPCommand>* tcp_commands;

public:

  /*! Stream IDs for different connection types */
  static const int8 cTCP_P2P_ID_EXPRESS = 9, cTCP_P2P_ID_BULK = 10;

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
  static tTCPCommand::tPtr GetUnusedTCPCommand();

  virtual void Init();

};

} // namespace finroc
} // namespace tcp

#endif // plugins__tcp__tTCP_h__
