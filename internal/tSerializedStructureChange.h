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
/*!\file    plugins/tcp/internal/tSerializedStructureChange.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-30
 *
 * \brief   Contains tSerializedStructureChange
 *
 * \b tSerializedStructureChange
 *
 * Whenever a structure change occurs, these changes are serialized and
 * stored within objects of this class in a concurrent queue to be processed
 * by the TCP thread.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tSerializedStructureChange_h__
#define __plugins__tcp__internal__tSerializedStructureChange_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Serialized changes to runtime structure
/*!
 * Whenever a structure change occurs, these changes are serialized and
 * stored within objects of this class in a concurrent queue to be processed
 * by the TCP thread.
 */
class tSerializedStructureChange : public rrlib::concurrent_containers::tQueueable<rrlib::concurrent_containers::tQueueability::MOST_OPTIMIZED>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \param change_type Type of change (see Constants)
   * \param element FrameworkElement that changed
   * \param serve_structure Does server serve full structure (if not, we do not need to serialize that)
   * \param minimum_relevant_level Level for which this structure change is relevant
   */
  tSerializedStructureChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element,
                             bool serve_structure, common::tStructureExchange minimum_relevant_level);

  /*!
   * \return Framework element's local handle
   */
  core::tFrameworkElement::tHandle GetLocalHandle() const
  {
    return local_handle;
  }

  /*!
   * \return Level for which this structure change is relevant
   */
  common::tStructureExchange MinimumRelevantLevel() const
  {
    return minimum_relevant_level;
  }

  /*!
   * Write structure change to specified output stream
   *
   * \param stream Stream to wrtie change to
   * \param exchange_level Exchange level to use
   */
  void WriteToStream(rrlib::serialization::tOutputStream& stream, common::tStructureExchange exchange_level) const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! What kind of change occured? */
  core::tRuntimeListener::tEvent change_type;

  /*!
   * Memory containing serialized information
   *
   * ADD:    [shared_port_info?][finstruct_structure_info]
   *           'full_structure_info_begin' points to before 'finstruct_structure_info'
   *           'finstruct_only_info_begin' splits 'finstruct_structure_info'
   * CHANGE: [finstruct_changeable_port_info]
   *           'full_structure_info_begin' points to zero
   *           'finstruct_only_info_begin' splits 'finstruct_changeable_port_info'
   * REMOVE:
   */
  rrlib::serialization::tFixedBuffer storage;

  /*! Level for which this structure change is relevant */
  common::tStructureExchange minimum_relevant_level;

  /*! Framework element's local handle */
  core::tFrameworkElement::tHandle local_handle;

  /*! Offset in memory buffer where serialized structure info begins */
  size_t full_structure_info_begin;

  /*! Offset in memory buffer where finstruct-only info begins */
  size_t finstruct_only_info_begin;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
