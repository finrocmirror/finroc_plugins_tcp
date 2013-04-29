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
/*!\file    plugins/tcp/internal/tMessage.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-12
 *
 * \brief   Contains tMessage
 *
 * \b tMessage
 *
 * Single message sent via the TCP protocol.
 * Typedef'ed for every opcode in in protocol_definitions.h.
 * This class is used mainly to serialize and deserialize data to the network streams.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tMessage_h__
#define __plugins__tcp__internal__tMessage_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/tSettings.h"
#include "plugins/tcp/internal/protocol_definitions.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace tcp
{
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

enum class tMessageSize
{
  FIXED,                   // fixed size message (calculated from TArgs)
  VARIABLE_UP_TO_255_BYTE, // variable message size up to 255 bytes
  VARIABLE_UP_TO_4GB       // variable message size up to 4GB
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Single TCP message
/*!
 * Single message sent via the TCP protocol.
 * Typedef'ed for every opcode in in protocol_definitions.h.
 * This class is used mainly to serialize and deserialize data to the network streams.
 *
 * \tparam OPCODE Opcode to message to send
 * \tparam SIZE Size of message (see enum)
 * \tparam TArgs Arguments of message
 */
template <tOpCode OPCODE, tMessageSize SIZE, typename ... TArgs>
struct tMessage : boost::noncopyable
{
  /*! parameters in this message */
  std::tuple<TArgs...> parameters;

  tMessage() :
    parameters()
  {}

  /*! Size of all arguments */
  template <bool ZERO_ARGS = (sizeof...(TArgs) == 0)>
  constexpr static typename std::enable_if < !ZERO_ARGS, int >::type ArgumentsSize()
  {
    return CalculateArgumentSize<TArgs...>();
  }
  template <bool ZERO_ARGS = (sizeof...(TArgs) == 0)>
  constexpr static typename std::enable_if<ZERO_ARGS, int>::type ArgumentsSize()
  {
    return 0;
  }

  /*! Helper for ArgumentsSize() */
  template <typename TArg>
  constexpr static int CalculateArgumentSize()
  {
    return (std::is_enum<TArg>::value ? 1 : sizeof(TArg));
  }
  template <typename TArg, typename TArg2, typename ... TRest>
  constexpr static int CalculateArgumentSize()
  {
    return (std::is_enum<TArg>::value ? 1 : sizeof(TArg)) + tMessage::CalculateArgumentSize<TArg2, TRest...>();
  }

  /*! Deserialize - excluding opcode and size */
  inline void Deserialize(rrlib::serialization::tInputStream& stream, bool finish = true)
  {
    stream >> parameters;
    if (finish)
    {
      FinishDeserialize(stream);
    }
  }

  inline void FinishDeserialize(rrlib::serialization::tInputStream& stream)
  {
    if (tSettings::cDEBUG_TCP)
    {
      uint8_t debug_number;
      stream >> debug_number;
      if (debug_number != tSettings::cDEBUG_TCP_NUMBER)
      {
        throw std::runtime_error("Message not correctly terminated");
      }
    }
  }

  static void FinishMessage(rrlib::serialization::tOutputStream& stream)
  {
    if (tSettings::cDEBUG_TCP)
    {
      stream << tSettings::cDEBUG_TCP_NUMBER;
    }
    if (SIZE != tMessageSize::FIXED)
    {
      stream.SkipTargetHere();
    }
  }

  /*!
   * \return Deserialized argument with specified index
   */
  template <size_t INDEX>
  inline typename std::tuple_element<INDEX, std::tuple<TArgs...>>::type& Get()
  {
    return std::get<INDEX>(parameters);
  }

  static constexpr tMessageSize MessageSize()
  {
    return SIZE;
  }

  /*!
   * Serialize - including opcode and size
   *
   * \param finish_message Is message complete after writing args? (If not, FinishMessage() must be called manually)
   */
  static inline void Serialize(bool finish_message, rrlib::serialization::tOutputStream& stream, const TArgs& ... args)
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Sending message ", make_builder::GetEnumString(OPCODE));
    if (OPCODE != tOpCode::OTHER)
    {
      stream << OPCODE;
    }
    if (SIZE != tMessageSize::FIXED)
    {
      stream.WriteSkipOffsetPlaceholder(SIZE == tMessageSize::VARIABLE_UP_TO_255_BYTE);
    }

    std::tuple<const TArgs& ...> args_tuple(args...);
    stream << args_tuple;
    if (finish_message)
    {
      FinishMessage(stream);
    }
  }


};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
