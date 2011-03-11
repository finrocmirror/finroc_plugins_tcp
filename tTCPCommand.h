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

#ifndef plugins__tcp__tTCPCommand_h__
#define plugins__tcp__tTCPCommand_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "core/portdatabase/tSerializableReusable.h"

namespace finroc
{
namespace core
{
class tCoreInput;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace tcp
{
/*!
 * \author Max Reichardt
 *
 * A single asynchronous TCP command such as: SUBSCRIBE or UNSUBSCRIBE
 */
class tTCPCommand : public core::tSerializableReusable
{
public:

  /*! OpCode - see TCP class */
  int8 op_code;

  /*! Handle of remote port */
  int remote_handle;

  /*! Strategy to use/request */
  int strategy;

  /*! Minimum network update interval */
  int16 update_interval;

  /*! Handle of local port */
  int local_index;

  /*! Data type uid */
  int16 datatypeuid;

  /*! Subscribe with reverse push strategy? */
  bool reverse_push;

  tTCPCommand() :
      op_code(0),
      remote_handle(0),
      strategy(0),
      update_interval(0),
      local_index(0),
      datatypeuid(0),
      reverse_push(false)
  {}

  virtual void Deserialize(core::tCoreInput& is)
  {
    throw util::tRuntimeException("Unsupported - not needed - server decodes directly (more efficient)", CODE_LOCATION_MACRO);
  }

  virtual void Serialize(core::tCoreOutput& os) const;

};

} // namespace finroc
} // namespace tcp

#endif // plugins__tcp__tTCPCommand_h__
