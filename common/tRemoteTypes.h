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
/*!\file    plugins/tcp/common/tRemoteTypes.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-13
 *
 * \brief   Contains tRemoteTypes
 *
 * \b tRemoteTypes
 *
 * This class aggregates information about types used in remote runtime environments.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__common__tRemoteTypes_h__
#define __plugins__tcp__common__tRemoteTypes_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/rtti/rtti.h"

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

/*!
 * Type of data type - mainly relevant for tooling
 */
enum tTypeClassification
{
  DATA_FLOW_STANDARD,
  DATA_FLOW_CHEAP_COPY,
  RPC_INTERFACE
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Information on types in remote runtime
/*!
 * This class aggregates information about types used in remote runtime environments.
 */
class tRemoteTypes : public rrlib::serialization::tTypeEncoder
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tRemoteTypes();

  /*!
   * \return Remote Global default update time
   */
  inline int16_t GetGlobalDefault() const
  {
    return global_default;
  }

  /*!
   * \param data_type Local Data Type
   * \return Remote default minimum network update interval for this type
   */
  int16_t GetTime(const rrlib::rtti::tType& data_type);

  /*!
   * \return Has this object been initialized?
   */
  inline bool Initialized() const
  {
    return types.size() != 0;
  }

  /*!
   * Set new update time for specified Type
   *
   * \param type_uid Type uid
   * \param new_time new update time
   */
  void SetTime(rrlib::rtti::tType dt, int16_t new_time);

  /*!
   * \return Have new types been added since last update?
   */
  inline bool TypeUpdateNecessary() const
  {
    return rrlib::rtti::tType::GetTypeCount() > local_types_sent;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Entry in remote type register */
  struct tEntry
  {
    tEntry();

    tEntry(rrlib::rtti::tType local);

    /*! local data type that represents the same time - null if there is no such type in local runtime environment */
    rrlib::rtti::tType local_data_type;

    /*! Number of local types checked to resolve type */
    int16_t types_checked;

    /*! name of remote type */
    std::string name;
  };

  /*! List with remote types - index is remote type id (=> mapping: remote type id => local type id) */
  std::vector<tEntry> types;

  /*! List with remote type update times - index is local type id */
  std::vector<int16_t> update_times;

  /*! Number (max index) of local types already sent to remote runtime */
  int16_t local_types_sent;

  /*! Remote Global default update time */
  int16_t global_default;

  /*!
   * Init remote data type information from input stream buffer.
   * (call only once!)
   *
   * \param ci Input Stream Buffer to read from
   */
  void Deserialize(rrlib::serialization::tInputStream& ci);

  virtual rrlib::rtti::tType ReadType(rrlib::serialization::tInputStream& is); // TODO: mark override in gcc 4.7

  /*!
   * Serializes information about local data types
   *
   * \param co Output Stream to write information to
   */
  void SerializeLocalDataTypes(rrlib::serialization::tOutputStream& co);

  virtual void WriteType(rrlib::serialization::tOutputStream& os, rrlib::rtti::tType dt); // TODO: mark override in gcc 4.7
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
