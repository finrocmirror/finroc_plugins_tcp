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
/*!\file    plugins/tcp/test/mTestCollection.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-20
 *
 */
//----------------------------------------------------------------------
#include "plugins/tcp/test/mTestCollection.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
namespace test
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<mTestCollection> cCREATE_ACTION("TestCollection");

static const int cINITIAL_VALUE = 50;

static tTestInterface test_interface;

rpc_ports::tRPCInterfaceType<tTestInterface> cTYPE("Test interface", &tTestInterface::Message, &tTestInterface::Function, &tTestInterface::FutureFunction);

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

mTestCollection::mTestCollection(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, true, true),
  out_initial_pushing(cINITIAL_VALUE),
  out_initial_pushing_reverse(tFlag::SHARED | tFlag::PUSH_STRATEGY_REVERSE),
  in_initial_pushing_reverse(cINITIAL_VALUE),
  in_burst_publishing(0, 5, tFlag::HAS_DEQUEUE_ALL_QUEUE),
  test_interface_client("Test Interface", &GetInputs()),
  test_interface_server(test_interface, "Test Interface", &GetOutputs(), tFlag::SHARED),
  float_blackboard("float blackboard", this, false, 10, true, blackboard::tReadPorts::INTERNAL),
  float_blackboard_client("float blackboard client", this, false, blackboard::tReadPorts::NONE),
  counter(0)
{
  in_pull_testing.SetPushStrategy(false);
  float_blackboard_client.GetOutsideWritePort().GetWrapped()->SetName("float blackboard");
}

void mTestCollection::Update()
{
  if (in_initial_pushing.Get() == cINITIAL_VALUE)
  {
//    if (counter < 50)
//    {
//      if (counter == 0)
//      {
//        FINROC_LOG_PRINT(USER, "Received value from initial pushing. Testing simple publishing:");
//        FINROC_LOG_PRINT(USER, "  Initial pushing: ", in_initial_pushing.Get(), "  Initial pushing reverse: ", out_initial_pushing_reverse.Get());
//      }
//      out_simple_publishing.Publish(counter);
//      FINROC_LOG_PRINT(USER, "  Received ", in_simple_publishing.Get());
//    }
//    else if (counter < 100)
//    {
//      if (counter == 50)
//      {
//        FINROC_LOG_PRINT(USER, "Testing burst publishing (10 values at once) with input queue of 5 values:");
//      }
//
//      for (size_t i = 0; i < 10; i++)
//      {
//        out_burst_publishing.Publish(counter * 10 + i);
//      }
//      std::ostringstream result_stream;
//      data_ports::tPortBuffers<int> dequeued = in_burst_publishing.DequeueAll();
//      while (!dequeued.Empty())
//      {
//        result_stream << dequeued.PopFront() << " ";
//      }
//      FINROC_LOG_PRINT(USER, "  Received port value burst: ", result_stream.str());
//    }
//    else if (counter < 150)
//    {
//      if (counter == 100)
//      {
//        FINROC_LOG_PRINT(USER, "Testing RPC calls:");
//
//        test_interface_client.Call(&tTestInterface::Message, "Hello Siegfried");
//        try
//        {
//          double result = test_interface_client.CallSynchronous(std::chrono::seconds(1), &tTestInterface::Function, 38);
//          FINROC_LOG_PRINT(USER, "  Called RPC function with 38. Result is ", result, ".");
//        }
//        catch (const rpc_ports::tRPCException& exception)
//        {
//          FINROC_LOG_PRINT(ERROR, "  Calling RPC function failed: ", exception);
//        }
//        try
//        {
//          rpc_ports::tFuture<int> result = test_interface_client.NativeFutureCall(&tTestInterface::FutureFunction);
//          FINROC_LOG_PRINT(USER, "  Called RPC future function. Result is ", result.Get(std::chrono::seconds(1)), ".");
//        }
//        catch (const rpc_ports::tRPCException& exception)
//        {
//          FINROC_LOG_PRINT(ERROR, "  Calling RPC future function failed: ", exception);
//        }
//
//        FINROC_LOG_PRINT(USER, "Testing blackboards:");
//      }
//
//      try
//      {
//        blackboard::tBlackboardWriteAccess<float> write_access(float_blackboard_client);
//        write_access[0] = counter - 100;
//      }
//      catch (const blackboard::tLockException& ex)
//      {
//        FINROC_LOG_PRINT(WARNING, "Could not lock remote blackboard for writing");
//      }
//
//      try
//      {
//        blackboard::tBlackboardWriteAccess<float> read_access(float_blackboard);
//        FINROC_LOG_PRINT(USER, "  First blackboard value in remote part: ", read_access[0]);
//      }
//      catch (const blackboard::tLockException& ex)
//      {
//        FINROC_LOG_PRINT(WARNING, "Could not lock remote blackboard for reading");
//      }
//    }

    if (counter < 50)
    {
      if (counter == 0)
      {
        FINROC_LOG_PRINT(USER, "Testing port value pulling:");
      }
      out_pull_testing.Publish(counter);
      FINROC_LOG_PRINT(USER, "  Received ", in_pull_testing.Get());
    }

    counter++;
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
