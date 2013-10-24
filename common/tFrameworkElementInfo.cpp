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
/*!\file    plugins/tcp/common/tFrameworkElementInfo.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-12
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/common/tFrameworkElementInfo.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tFrameworkElementTags.h"
#include "plugins/data_ports/common/tAbstractDataPort.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
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
typedef core::tFrameworkElement::tFlag tFlag;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

tFrameworkElementInfo::tFrameworkElementInfo() :
  links(),
  link_count(0),
  handle(0),
  type(),
  changeable_info()
{}

void tFrameworkElementInfo::Deserialize(rrlib::serialization::tInputStream& stream)
{
  link_count = stream.ReadByte();
  for (int i = 0; i < link_count; i++)
  {
    if (i < cMAX_LINKS)
    {
      std::string name = stream.ReadString();
      links[i] = { name, stream.ReadBoolean() };
    }
    else
    {
      FINROC_LOG_PRINT(WARNING, "More than ", cMAX_LINKS, " received. Skipping additional ones.");
    }
  }

  stream >> type >> changeable_info;
}

void tFrameworkElementInfo::Serialize(rrlib::serialization::tOutputStream& stream, core::tFrameworkElement& framework_element,
                                      tStructureExchange structure_exchange_level, std::string& string_buffer)
{
  if (structure_exchange_level == tStructureExchange::NONE)
  {
    FINROC_LOG_PRINT(WARNING, "Specifying structure exchange level tStructureExchange::NONE does not write anything to stream. This is typically not intended.");
    return;
  }

  // serialize handle?
  //stream << framework_element.GetHandle();

  // serialize links
  int link_count = framework_element.GetLinkCount();
  assert(link_count < 128); // is guaranteed by tFrameworkElement
  int port_flag = 0;
  if ((structure_exchange_level != tStructureExchange::SHARED_PORTS) && framework_element.IsPort())
  {
    port_flag = 0x80; // flag ports
  }
  stream.WriteByte(link_count | port_flag);
  for (int i = 0; i < link_count; i++)
  {
    if (structure_exchange_level == tStructureExchange::SHARED_PORTS)
    {
      bool unique = framework_element.GetQualifiedLink(string_buffer, i);
      stream << (string_buffer.c_str() + 1) << unique;  // omit first slash
    }
    else
    {
      bool unique = framework_element.GetFlag(tFlag::GLOBALLY_UNIQUE_LINK) || framework_element.GetParentWithFlags(tFlag::GLOBALLY_UNIQUE_LINK);
      core::tFrameworkElement* parent = framework_element.GetParent(i);
      stream << framework_element.GetLink(i)->GetName() << unique << parent->GetHandle();
    }
  }

  // send additional info - depending on whether we have a port
  if (!framework_element.IsPort())
  {
    stream << framework_element.GetAllFlags().Raw(); // TODO: We could save 2 bytes - as not all flags are relevant (first + last 8 bits are sufficient for ordinary framework elements)
  }
  else
  {
    core::tAbstractPort& port = static_cast<core::tAbstractPort&>(framework_element);
    stream << port.GetDataType() << port.GetAllFlags().Raw();
    if (data_ports::IsDataFlowType(port.GetDataType()))
    {
      data_ports::common::tAbstractDataPort& data_port = static_cast<data_ports::common::tAbstractDataPort&>(port);
      stream << data_port.GetStrategy() << data_port.GetMinNetUpdateIntervalRaw();
    }
    else
    {
      // Serialize the two dummy values for strategy and update interval
      // - as destination might not know whether the type is actually a data flow type.
      // As > 90& of ports are data flow ports, putting extra information on type in the stream would consume more bandwidth
      uint16_t unused = 0;
      stream << unused << unused;
    }
  }

  if (structure_exchange_level == tStructureExchange::FINSTRUCT)
  {
    SerializeFinstructOnlyInfo(stream, framework_element);
  }
}

void tFrameworkElementInfo::SerializeConnections(rrlib::serialization::tOutputStream& stream, core::tAbstractPort& port)
{
  std::array<core::tAbstractPort*, 256> connections;
  int connection_count = 0;
  for (auto it = port.OutgoingConnectionsBegin(); it != port.OutgoingConnectionsEnd(); ++it)
  {
    connections[connection_count] = &(*it);
    if (connection_count == 255)
    {
      FINROC_LOG_PRINT(WARNING, "Port ", port, " has more than 255 connections. Serializing only 255. The rest will not be visible in finstruct.");
      break;
    }
    connection_count++;
  }
  stream.WriteByte(connection_count);
  for (int i = 0; i < connection_count; i++)
  {
    stream.WriteInt(connections[i]->GetHandle());
    stream.WriteBoolean(port.IsEdgeFinstructed(*connections[i]));
  }
}

void tFrameworkElementInfo::SerializeFinstructOnlyInfo(rrlib::serialization::tOutputStream& stream, core::tFrameworkElement& framework_element)
{
  // serialize connections?
  if (framework_element.IsPort())
  {
    SerializeConnections(stream, static_cast<core::tAbstractPort&>(framework_element));
  }

  // possibly send tags
  core::tFrameworkElementTags* tags = framework_element.GetAnnotation<core::tFrameworkElementTags>();
  stream.WriteBoolean(tags);
  if (tags)
  {
    stream << (*tags);
  }
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
