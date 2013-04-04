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
/*!\file    plugins/tcp/common/tFrameworkElementInfo.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-12
 *
 * \brief   Contains tFrameworkElementInfo
 *
 * \b tFrameworkElementInfo
 *
 * Information on (shared) framework elements - as exchanged by peers.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__common__tFrameworkElementInfo_h__
#define __plugins__tcp__common__tFrameworkElementInfo_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/port/tAbstractPort.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/common/tChangeablePortInfo.h"

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

/*!
 * Enum on different levels of structure (framework elements and ports) exchanged among peers
 */
enum class tStructureExchange : int
{
  NONE, //<! No structure info on structure is sent
  SHARED_PORTS, //<! Send info on shared ports to connection partner
  COMPLETE_STRUCTURE, //<! Send info on complete structure to connection partner (e.g. for fingui)
  FINSTRUCT, //<! Send info on complete structure including port connections to partner (as required by finstruct)
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Framework element information
/*!
 * Information on (shared) framework elements - as exchanged by peers.
 *
 * In C++, this struct can only store info shared ports.
 * The static Serialize method also serializes framework element info
 * and port connection info.
 */
struct tFrameworkElementInfo
{
  typedef typename core::tFrameworkElement::tHandle tHandle;
  typedef typename core::tFrameworkElement::tFlag tFlag;
  typedef typename core::tFrameworkElement::tFlags tFlags;

  /*!
   * Infos regarding links to this element
   */
  struct tLinkInfo
  {
    /*! name */
    std::string name;

    /*! Port with globally unique UID? */
    bool unique;
  };

  /*! Maximum number of links per port */
  enum { cMAX_LINKS = 3 };

  /*! Information about links to this port - currently in fixed array for efficiency reasons */
  std::array<tLinkInfo, cMAX_LINKS> links;

  /*! Number of links */
  uint8_t link_count;

  /*! Handle in runtime environment */
  tHandle handle;

  /*! Type of port data */
  rrlib::rtti::tType type;

  /*! Inconstant port information */
  tChangeablePortInfo changeable_info;


  tFrameworkElementInfo();

  /*!
   * Deserializes info on single framework element with structure exchange
   * level SHARED_PORTS.
   *
   * \param stream Binary stream to deserialize from
   */
  void Deserialize(rrlib::serialization::tInputStream& stream);

  /*!
   * Serializes info on single framework element to stream so that it can later
   * be deserialized in typically another runtime environment.
   *
   * \param stream Binary stream to serialize to
   * \param framework_element Framework element to serialize info of
   * \param structure_exchange_level Determines how much information is serialized
   * \param string_buffer Temporary string buffer
   */
  static void Serialize(rrlib::serialization::tOutputStream& stream, core::tFrameworkElement& framework_element,
                        tStructureExchange structure_exchange_level, std::string& string_buffer);

  /*!
   * Serializes connections of specified port
   *
   * \param stream Binary stream to serialize to
   * \param port Port to serialize connections of
   */
  static void SerializeConnections(rrlib::serialization::tOutputStream& stream, core::tAbstractPort& port);

  /*!
   * Serializes information that is send to "finstruct clients" in addition to the data all "structure clients" receive.
   *
   * That means, calling
   *
   *   Serialize(stream, framework_element, tStructureExchange::COMPLETE_STRUCTURE, string_buffer);
   *   SerializeFinstructOnlyInfo(stream, framework_element);
   *
   * is equivalent to calling
   *
   *   Serialize(stream, framework_element, tStructureExchange::FINSTRUCT, string_buffer);
   *
   * \param stream Binary stream to serialize to
   * \param port Port to serialize connections of
   */
  static void SerializeFinstructOnlyInfo(rrlib::serialization::tOutputStream& stream, core::tFrameworkElement& framework_element);
};

//inline rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tFrameworkElementInfo& info)
//{
//  info.Serialize(stream);
//  return stream;
//}

inline rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tFrameworkElementInfo& info)
{
  info.Deserialize(stream);
  return stream;
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
