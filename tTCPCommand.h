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

#include "rrlib/serialization/serialization.h"
#include "rrlib/rtti/tDataTypeBase.h"

#include "core/portdatabase/tSerializableReusable.h"


namespace finroc
{
namespace tcp
{

/**
 * Protocol OpCodes
 *
 * TODO: protocol could be optimized/simplified
 *
 * payload is   [int8: data encoding]
 *            n*[bool8 another buffer? == true][int16: data type uid][int8: encoding][binary blob or null-terminated string depending on type encoding]
 *              [bool8 another buffer? == false]
 *
 * parameter is [int8: parameter type? == NULL_PARAM]
 *              [int8: parameter type? == NUMBER][serialized tCoreNumber]
 *              [int8: parameter_type? == OBJECT][int16: data type][serialized data][bool8: write lockid? if TRUE, followed by int32 lock id]
 */
enum class tOpCode
{
  SET,               // Port data set operation             [int32: remote port handle][int32: skip offset][int8: changed flag][payload](skip target)
  SUBSCRIBE,         // Subscribe to data port              [int32: remote port handle][int16: strategy][bool: reverse push][int16: update interval][int32: local port handle][int8: encoding]
  UNSUBSCRIBE,       // Unsubscribe from data port          [int32: remote port handle]
  CHANGE_EVENT,      // Change event from subscription      [int32: remote port handle][int32: skip offset][int8: changed flag][payload](skip target)
  PING,              // Ping to determine round-trip time   [int32: packet index]
  PONG,              // Pong to determine round-trip time   [int32: acknowledged packet index]
  PULLCALL,          // Pull call                           [int32: remote port handle][int32: local port handle][int32: skip offset][int8: status][int8: exception type][int8: syncher ID][int32: thread uid][int16: method call index][bool8 intermediateAssign][int8: desired encoding](skip target)
  PULLCALL_RETURN,   // Returning pull call                 [int32: remote port handle][int32: local port handle][int32: skip offset][int8: status][int8: exception type][int8: syncher ID][int32: thread uid][int16: method call index][bool8 intermediateAssign][int8: desired encoding][int16: data type][serialized data](skip target)
  METHODCALL,        // Method call                         [int32: remote port handle][int32: local port handle][int16: interface type][int32: skip offset][int8: method id][int32: timeout in ms][int8: status][int8: exception type][int8: syncher ID][int32: thread uid][int16: method call index][parameter0][parameter1][parameter2][parameter3](skip target)
  METHODCALL_RETURN, // Returning Method call               [int32: remote port handle][int32: local port handle][int16: interface type][int32: skip offset][int8: method id][int32: timeout in ms][int8: status][int8: exception type][int8: syncher ID][int32: thread uid][int16: method call index][parameter0][parameter1][parameter2][parameter3](skip target)
  UPDATE_TIME,       // Update on desired update times      [int16: data type][int16: new update time]
  STRUCTURE_UPDATE,  // Update on remote framework elements [int8: opcode (e.g. add)][int32: remote handle][int32: flags][bool8: only ports?]... (see serializeFrameworkElement() in FrameworkElementInfo)
  PEER_INFO          // Information about other peers       [int32+int16: IP and port of this runtime in partner's network interface][int32: known peer count n] n*[int32+int16: address of peer[i]]
};


/*!
 * \author Max Reichardt
 *
 * A single asynchronous TCP command such as: SUBSCRIBE or UNSUBSCRIBE
 */
class tTCPCommand : public core::tSerializableReusable
{
public:

  typedef std::unique_ptr<tTCPCommand, tRecycler> tPtr;

  /*! OpCode - see TCP class */
  tOpCode op_code;

  /*! Handle of remote port */
  int remote_handle;

  /*! Strategy to use/request */
  int strategy;

  /*! Minimum network update interval */
  int16 update_interval;

  /*! Handle of local port */
  int local_index;

  /*! Data type uid */
  rrlib::rtti::tDataTypeBase datatype;

  /*! Subscribe with reverse push strategy? */
  bool reverse_push;

  /*! Data encoding to use */
  rrlib::serialization::tDataEncoding encoding;

  tTCPCommand() :
    op_code(tOpCode::SET),
    remote_handle(0),
    strategy(0),
    update_interval(0),
    local_index(0),
    datatype(),
    reverse_push(false),
    encoding(rrlib::serialization::tDataEncoding::BINARY)
  {}

  virtual void Deserialize(rrlib::serialization::tInputStream& is)
  {
    throw util::tRuntimeException("Unsupported - not needed - server decodes directly (more efficient)", CODE_LOCATION_MACRO);
  }

  virtual void Serialize(rrlib::serialization::tOutputStream& os) const;

  virtual void CustomDelete(bool b)
  {
    tReusable::CustomDelete(b);
  }

};

} // namespace finroc
} // namespace tcp

#endif // plugins__tcp__tTCPCommand_h__
