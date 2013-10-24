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
/*!\file    plugins/tcp/test/mTestCollection.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-20
 *
 * \brief   Contains mTestCollection
 *
 * \b mTestCollection
 *
 * Module that performs various tests if its ports are connected to another
 * instance of this module.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__test__mTestCollection_h__
#define __plugins__tcp__test__mTestCollection_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/blackboard/tBlackboard.h"

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
namespace test
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

class tTestInterface : public rpc_ports::tRPCInterface
{
public:
  void Message(const std::string& message)
  {
    FINROC_LOG_PRINT(USER, "  Received string message via RPC: ", message);
  }

  double Function(double d)
  {
    return d + 4.0;
  }

  rpc_ports::tFuture<int> FutureFunction()
  {
    rpc_ports::tPromise<int> promise;
    promise.SetValue(42);
    return promise.GetFuture();
  }
};

extern rpc_ports::tRPCInterfaceType<tTestInterface> cTYPE;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Test module
/*!
 * Module that performs various tests if its ports are connected to another
 * instance of this module.
 */
class mTestCollection : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tOutput<int> out_initial_pushing;
  tInput<int> in_initial_pushing;

  tOutput<int> out_initial_pushing_reverse;
  tInput<int> in_initial_pushing_reverse;

  tOutput<int> out_simple_publishing;
  tInput<int> in_simple_publishing;

  tOutput<int> out_burst_publishing;
  tInput<int> in_burst_publishing;

  tOutput<int> out_pull_testing;
  tInput<int> in_pull_testing;

  rpc_ports::tClientPort<tTestInterface> test_interface_client;
  rpc_ports::tServerPort<tTestInterface> test_interface_server;

  blackboard::tBlackboard<float> float_blackboard;
  blackboard::tBlackboardClient<float> float_blackboard_client;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mTestCollection(core::tFrameworkElement *parent, const std::string &name = "TestCollection");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  uint32_t counter;

  virtual void Update();
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
