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
/*!\file    plugins/tcp/common/tRemoteTypes.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-13
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/common/tRemoteTypes.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/log_messages.h"
#include "plugins/rpc_ports/definitions.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/common/tNetworkUpdateTimeSettings.h"

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

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

tRemoteTypes::tRemoteTypes() :
  types(),
  update_times(),
  local_types_sent(0),
  global_default(0)
{
}

void tRemoteTypes::Deserialize(rrlib::serialization::tInputStream& ci)
{
  std::stringstream ls;
  if (types.size() == 0)
  {
    global_default = ci.ReadShort();
    ls << "Connection Partner knows types:" << std::endl;
  }
  else
  {
    ls << "Connection Partner knows more types:" << std::endl;
  }
  int16_t next = ci.ReadShort();
  while (next != -1)
  {
    int16_t time = ci.ReadShort();

    ci.ReadEnum<tTypeClassification>();

    std::string name = ci.ReadString();
    int16_t checked_types = rrlib::rtti::tType::GetTypeCount();
    rrlib::rtti::tType local = rrlib::rtti::tType::FindType(name);
    ls << "- " << name << " (" << next << ") - " << (local != NULL ? "available here, too" : "not available here") << std::endl;
    __attribute__((unused)) // prevents warning in release mode
    int types_size = types.size();  // to avoid warning
    assert(next == types_size);
    tEntry e(local);
    e.types_checked = checked_types;

    if (local == NULL)
    {
      e.name = name;
    }

    // read additional data we do not need in C++ (remote type traits and enum constant names)
    int8_t traits = ci.ReadByte(); // type traits
    if (traits & rrlib::rtti::trait_flags::cIS_ENUM)
    {
      short n = ci.ReadShort();
      for (short i = 0; i < n; i++)
      {
        ci.SkipString();
      }
    }

    types.push_back(e);
    if (local != NULL)
    {
      if (update_times.size() < rrlib::rtti::tType::GetTypeCount())
      {
        update_times.resize(rrlib::rtti::tType::GetTypeCount() + 10, -1);
      }
      update_times[local.GetUid()] = time;
    }
    next = ci.ReadShort();
  }
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, ls.str());
}

int16_t tRemoteTypes::GetTime(const rrlib::rtti::tType& data_type)
{
  if (!Initialized())
  {
    throw std::runtime_error("No types initialized yet");
  }
  if (update_times.size() < rrlib::rtti::tType::GetTypeCount())
  {
    update_times.resize(rrlib::rtti::tType::GetTypeCount() + 10, -1);
  }
  return update_times[data_type.GetUid()];
}

rrlib::rtti::tType tRemoteTypes::ReadType(rrlib::serialization::tInputStream& is)
{
  int16_t uid = is.ReadShort();
  if (uid == -2)
  {
    // we get info on more data
    Deserialize(is);
    uid = is.ReadShort();
  }

  if (!Initialized())
  {
    throw std::runtime_error("No types initialized yet");
  }

  int types_size = types.size();  // to avoid warning
  if (uid < 0 || uid >= types_size)
  {
    FINROC_LOG_PRINT(ERROR, "Corrupt type information from received from connection partner: ", uid);
    throw std::runtime_error("Corrupt type information from received from connection partner");
  }

  tEntry& e = types[uid];
  if (e.local_data_type == NULL && e.types_checked < rrlib::rtti::tType::GetTypeCount())
  {
    // we have more types locally... maybe we can resolve missing types now
    e.types_checked = rrlib::rtti::tType::GetTypeCount();
    e.local_data_type = rrlib::rtti::tType::FindType(e.name);
  }
  return e.local_data_type;
}

void tRemoteTypes::SerializeLocalDataTypes(rrlib::serialization::tOutputStream& co)
{
  if (local_types_sent == 0)
  {
    int t = tNetworkUpdateTimeSettings::GetInstance().default_minimum_network_update_time.Get();
    co.WriteShort(static_cast<int16_t>(t));
  }
  int16_t type_count = rrlib::rtti::tType::GetTypeCount();
  for (int16_t i = local_types_sent, n = type_count; i < n; i++)
  {
    rrlib::rtti::tType dt = rrlib::rtti::tType::GetType(i);

    co.WriteShort(dt.GetUid());
    co.WriteShort(tNetworkUpdateTimeSettings::GetMinNetUpdateInterval(dt));
    tTypeClassification classification = rpc_ports::IsRPCType(dt) ? tTypeClassification::RPC_INTERFACE :
                                         (data_ports::IsCheaplyCopiedType(dt) ? tTypeClassification::DATA_FLOW_STANDARD : tTypeClassification::DATA_FLOW_CHEAP_COPY);
    co << classification;
    co.WriteString(dt.GetName());
    const int cTRAITS = rrlib::rtti::trait_flags::cIS_ENUM | rrlib::rtti::trait_flags::cIS_BINARY_SERIALIZABLE | rrlib::rtti::trait_flags::cIS_STRING_SERIALIZABLE | rrlib::rtti::trait_flags::cIS_XML_SERIALIZABLE;
    static_assert(cTRAITS <= 255, "Does not fit into byte");
    co.WriteByte(dt.GetTypeTraits() & cTRAITS); // type traits
    if ((dt.GetTypeTraits() & rrlib::rtti::trait_flags::cIS_ENUM) != 0)
    {
      const char* const* enum_strings = dt.GetEnumStrings();
      size_t enum_strings_dimension = dt.GetEnumStringsDimension();
      co.WriteShort(enum_strings_dimension);
      for (size_t j = 0; j < enum_strings_dimension; j++)
      {
        co.WriteString(enum_strings[j]);
      }
    }
  }
  co.WriteShort(-1);  // terminator
  local_types_sent = type_count;
}

void tRemoteTypes::SetTime(rrlib::rtti::tType dt, int16_t new_time)
{
  if (!Initialized())
  {
    throw std::runtime_error("No types initialized yet");
  }

  if (dt == NULL)
  {
    assert((new_time >= 0));
    global_default = new_time;
  }
  else
  {
    if (update_times.size() < rrlib::rtti::tType::GetTypeCount())
    {
      update_times.resize(rrlib::rtti::tType::GetTypeCount() + 10, -1);
    }
    update_times[dt.GetUid()] = new_time;
  }
}

void tRemoteTypes::WriteType(rrlib::serialization::tOutputStream& os, rrlib::rtti::tType dt)
{
  int count = rrlib::rtti::tType::GetTypeCount();
  if (count > local_types_sent)
  {
    os.WriteShort(-2);
    SerializeLocalDataTypes(os);
  }
  os.WriteShort(dt.GetUid());
}

tRemoteTypes::tEntry::tEntry() :
  local_data_type(),
  types_checked(0),
  name()
{}

tRemoteTypes::tEntry::tEntry(rrlib::rtti::tType local) :
  local_data_type(local),
  types_checked(0),
  name()
{
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
