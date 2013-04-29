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
/*!\file    plugins/tcp/internal/tSerializedStructureChange.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-30
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tSerializedStructureChange.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/util.h"

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
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

tSerializedStructureChange::tSerializedStructureChange(core::tRuntimeListener::tEvent change_type,
    core::tFrameworkElement& element, bool serve_structure, common::tStructureExchange minimum_relevant_level) :
  change_type(change_type),
  storage(0),
  minimum_relevant_level(minimum_relevant_level),
  local_handle(element.GetHandle()),
  full_structure_info_begin(0),
  finstruct_only_info_begin(0)
{
  if (change_type == core::tRuntimeListener::tEvent::ADD)
  {
    rrlib::serialization::tStackMemoryBuffer<4096> buffer;
    rrlib::serialization::tOutputStream stream(buffer);
    std::string string_buffer;

    if (minimum_relevant_level == common::tStructureExchange::SHARED_PORTS)
    {
      common::tFrameworkElementInfo::Serialize(stream, element, common::tStructureExchange::SHARED_PORTS, string_buffer);
      stream.Flush();
      full_structure_info_begin = buffer.GetSize();
    }
    //if (serve_structure)  if we start serving structure, while this is still in a queue... this is suboptimal
    //{
    common::tFrameworkElementInfo::Serialize(stream, element, common::tStructureExchange::COMPLETE_STRUCTURE, string_buffer);
    stream.Flush();
    finstruct_only_info_begin = buffer.GetSize();
    common::tFrameworkElementInfo::SerializeFinstructOnlyInfo(stream, element);
    stream.Flush();
    //}
    storage = CopyToNewFixedBuffer(buffer);
    //FINROC_LOG_PRINT(DEBUG, this, " ", full_structure_info_begin, " ", finstruct_only_info_begin);
  }
  else if (change_type == core::tRuntimeListener::tEvent::CHANGE)
  {
    common::tChangeablePortInfo info;
    info.flags = element.GetAllFlags();
    assert(element.IsPort());
    core::tAbstractPort& port = static_cast<core::tAbstractPort&>(element);
    if (data_ports::IsDataFlowType(port.GetDataType()))
    {
      data_ports::common::tAbstractDataPort& data_port = static_cast<data_ports::common::tAbstractDataPort&>(element);
      info.min_net_update_time = data_port.GetMinNetUpdateIntervalRaw();
      info.strategy = data_port.GetStrategy();
    }

    rrlib::serialization::tStackMemoryBuffer<2048> buffer;
    rrlib::serialization::tOutputStream stream(buffer);
    stream << info;
    stream.Flush();
    finstruct_only_info_begin = buffer.GetSize();
    if (element.IsPort() && serve_structure)
    {
      common::tFrameworkElementInfo::SerializeConnections(stream, port);
      stream.Flush();
    }
    storage = CopyToNewFixedBuffer(buffer);
  }
}

void tSerializedStructureChange::WriteToStream(rrlib::serialization::tOutputStream& stream, common::tStructureExchange exchange_level) const
{
  //FINROC_LOG_PRINT(DEBUG, this, " ", full_structure_info_begin, " ", finstruct_only_info_begin);

  typedef std::pair<const void*, size_t> tBufferInfo;
  if (change_type == core::tRuntimeListener::tEvent::ADD)
  {
    tStructureCreateMessage::Serialize(false, stream, local_handle);
    std::array<tBufferInfo, 4> buffers = {{
        tBufferInfo(NULL, 0),
        tBufferInfo(storage.GetPointer(), full_structure_info_begin), // shared pointers
        tBufferInfo(storage.GetPointer() + full_structure_info_begin, finstruct_only_info_begin - full_structure_info_begin), // complete structure
        tBufferInfo(storage.GetPointer() + full_structure_info_begin, storage.Capacity() - full_structure_info_begin) // finstruct
      }
    };
    tBufferInfo& buffer = buffers[static_cast<size_t>(exchange_level)];
    stream.Write(buffer.first, buffer.second);
    tStructureCreateMessage::FinishMessage(stream);
  }
  else if (change_type == core::tRuntimeListener::tEvent::CHANGE)
  {
    tStructureChangeMessage::Serialize(false, stream, local_handle);
    stream.Write(storage.GetPointer(), exchange_level == common::tStructureExchange::FINSTRUCT ? storage.Capacity() : finstruct_only_info_begin);
    tStructureCreateMessage::FinishMessage(stream);
  }
  else if (change_type == core::tRuntimeListener::tEvent::REMOVE)
  {
    tStructureDeleteMessage::Serialize(true, stream, local_handle);
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
