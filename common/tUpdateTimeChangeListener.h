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
/*!\file    plugins/tcp/common/tUpdateTimeChangeListener.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-13
 *
 * \brief   Contains tUpdateTimeChangeListener
 *
 * \b tUpdateTimeChangeListener
 *
 * Gets notified on changes to global default minimum network update times
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__common__tUpdateTimeChangeListener_h__
#define __plugins__tcp__common__tUpdateTimeChangeListener_h__

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Listens to update time changes
/*!
 * Gets notified on changes to global default minimum network update times
 */
class tUpdateTimeChangeListener
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * Called whenever default update time globally or for specific type changes
   *
   * \param dt DataType - null for global change
   * \param new_update_time new update time
   */
  virtual void UpdateTimeChanged(rrlib::rtti::tType dt, int16_t new_update_time) = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
