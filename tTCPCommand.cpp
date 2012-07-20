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
#include "plugins/tcp/tTCPCommand.h"
#include "rrlib/serialization/tOutputStream.h"
#include "plugins/tcp/tTCP.h"

namespace finroc
{
namespace tcp
{
void tTCPCommand::Serialize(rrlib::serialization::tOutputStream& os) const
{
  os.WriteEnum(op_code);
  switch (op_code)
  {
  case tOpCode::SUBSCRIBE:
    os.WriteInt(remote_handle);
    os.WriteShort(strategy);
    os.WriteBoolean(reverse_push);
    os.WriteShort(update_interval);
    os.WriteInt(local_index);
    os.WriteEnum(encoding);
    break;
  case tOpCode::UNSUBSCRIBE:
    os.WriteInt(remote_handle);
    break;
  case tOpCode::UPDATE_TIME:
    os << datatype;
    os.WriteShort(update_interval);
    break;
  default:
    FINROC_LOG_PRINTF(ERROR, "Invalid opcode");
    break;
  }
}

} // namespace finroc
} // namespace tcp

