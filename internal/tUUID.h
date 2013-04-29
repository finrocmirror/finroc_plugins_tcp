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
/*!\file    plugins/tcp/internal/tUUID.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 * \brief   Contains tUUID
 *
 * \b tUUID
 *
 * UUID of TCP peer in network.
 * Currently consists of host name and network port - which seems optimal and simple.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tUUID_h__
#define __plugins__tcp__internal__tUUID_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/serialization/serialization.h"
#include <boost/lexical_cast.hpp>

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
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! TCP Peer UUID
/*!
 * UUID of TCP peer in network.
 * Currently consists of host name and network port - which seems optimal and simple.
 */
struct tUUID
{
  /*! Host name */
  std::string host_name;

  /** Server Port (positive numbers) or process id (negative number; for clients) */
  int port;

  tUUID() : host_name(), port(-1) {}

  bool operator==(const tUUID& other) const
  {
    return host_name == other.host_name && port == other.port;
  }
  bool operator!=(const tUUID& other) const
  {
    return !operator==(other);
  }

  std::string ToString() const
  {
    if (port >= 0)
    {
      return host_name + ":" + boost::lexical_cast<std::string>(port);
    }
    return host_name + "<" + boost::lexical_cast<std::string>(-port) + ">";
  }
};

inline rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tUUID& uuid)
{
  stream << uuid.host_name << uuid.port;
  return stream;
}

inline rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tUUID& uuid)
{
  stream >> uuid.host_name >> uuid.port;
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
