//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/tcp/common/tChangeablePortInfo.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-21
 *
 * \brief   Contains tChangeablePortInfo
 *
 * \b tChangeablePortInfo
 *
 * Information about ports that may change during application runtime
 * (e.g. push/pull strategy, network update times)
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__common__tChangeablePortInfo_h__
#define __plugins__tcp__common__tChangeablePortInfo_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace tcp
{
namespace common
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Inconstant port information
/*!
 * Information about ports that may change during application runtime
 * (e.g. push/pull strategy, network update times)
 */
struct tChangeablePortInfo
{
  /*! Framework element Flags (only first 8 bit are serialized in case of non-ports) */
  core::tFrameworkElement::tFlags flags;

  /*! Strategy to use for this port - if it is destination port */
  int16_t strategy;

  /*! Minimum network update interval */
  int16_t min_net_update_time;


  tChangeablePortInfo() :
    flags(),
    strategy(0),
    min_net_update_time(-1)
  {}
};

inline rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tChangeablePortInfo& info)
{
  stream << info.flags.Raw() << info.strategy << info.min_net_update_time;
  return stream;
}

inline rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tChangeablePortInfo& info)
{
  uint32_t flags;
  stream >> flags;
  info.flags = core::tFrameworkElement::tFlags(flags);
  stream >> info.strategy >> info.min_net_update_time;
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
