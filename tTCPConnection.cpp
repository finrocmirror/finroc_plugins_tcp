/**
 * You received this file as part of an advanced experimental
 * robotics framework prototype ('finroc')
 *
 * Copyright (C) 2007-2010 Max Reichardt,
 *   Robotics Research Lab, University of Kaiserslautern
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include "plugins/tcp/tTCPConnection.h"
#include "core/tRuntimeSettings.h"
#include "plugins/tcp/tTCPPeer.h"
#include "rrlib/finroc_core_utils/tTime.h"
#include "core/portdatabase/tFinrocTypeInfo.h"
#include "core/port/rpc/tMethodCall.h"
#include "core/port/tThreadLocalCache.h"
#include "core/port/rpc/tMethodCallException.h"
#include "core/port/tAbstractPort.h"
#include "core/port/net/tNetPort.h"
#include "core/port/rpc/tPullCall.h"
#include "core/port/rpc/tRPCThreadPool.h"
#include "plugins/tcp/tTCPCommand.h"
#include "rrlib/finroc_core_utils/thread/tLoopThread.h"
#include "plugins/tcp/tPeerList.h"
#include "rrlib/finroc_core_utils/net/tIPAddress.h"
#include "rrlib/finroc_core_utils/net/tIPSocketAddress.h"
#include "core/portdatabase/tSerializableReusable.h"
#include "rrlib/finroc_core_utils/container/tAbstractReusable.h"

namespace finroc
{
namespace tcp
{
tTCPConnection::tTCPConnection(int8 type_, tTCPPeer* peer_, bool send_peer_info_to_partner_) :
    min_update_interval((type_ == tTCP::cTCP_P2P_ID_BULK) ? tTCPSettings::GetInstance()->min_update_interval_bulk : tTCPSettings::GetInstance()->min_update_interval_express),
    max_not_acknowledged_packets((type_ == tTCP::cTCP_P2P_ID_BULK) ? tTCPSettings::GetInstance()->max_not_acknowledged_packets_bulk : tTCPSettings::GetInstance()->max_not_acknowledged_packets_express),
    last_acknowledged_packet(0),
    last_ack_request_index(0),
    sent_packet_time(tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS + 1),
    ping_times(tTCPSettings::cAVG_PING_PACKETS + 1),
    disconnect_signal(false),
    socket(),
    cos(),
    cis(),
    writer(),
    reader(),
    time_base(0),
    update_times(),
    type(type_),
    monitored_ports(50u, 4u),
    peer(peer_),
    send_peer_info_to_partner(send_peer_info_to_partner_),
    last_peer_info_sent_revision(-1),
    last_rx_timestamp(0),
    last_rx_position(0),
    obj_mutex(core::tLockOrderLevels::cREMOTE + 1)
{
  core::tRuntimeSettings::GetInstance()->AddUpdateTimeChangeListener(this);
  if (peer_ != NULL)
  {
    peer_->AddConnection(this);
  }
}

int64 tTCPConnection::CheckPingForDisconnect()
{
  std::shared_ptr<tWriter> locked_writer = writer.lock();
  if (locked_writer == NULL)
  {
    return tTCPSettings::GetInstance()->critical_ping_threshold.GetValue();
  }
  if (last_acknowledged_packet != locked_writer->cur_packet_index)
  {
    int64 critical_packet_time = sent_packet_time[(last_acknowledged_packet + 1) & tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS];
    int64 time_left = critical_packet_time + tTCPSettings::GetInstance()->critical_ping_threshold.GetValue() - util::tSystem::CurrentTimeMillis();
    if (time_left < 0)
    {
      HandlePingTimeExceed();
      return tTCPSettings::GetInstance()->critical_ping_threshold.GetValue();
    }
    return time_left;
  }
  else
  {
    return tTCPSettings::GetInstance()->critical_ping_threshold.GetValue();
  }
}

void tTCPConnection::Disconnect()
{
  util::tLock lock1(this);
  disconnect_signal = true;
  core::tRuntimeSettings::GetInstance()->RemoveUpdateTimeChangeListener(this);
  if (peer != NULL)
  {
    peer->RemoveConnection(this);
  }
  NotifyWriter();  // stops writer
  //cos = null;
  try
  {
    socket->Close();  // stops reader
  }
  catch (const util::tException& e)
  {
  }

  // join threads for thread safety
  std::shared_ptr<tWriter> locked_writer = writer.lock();
  if (locked_writer != NULL && util::tThread::CurrentThread() != locked_writer)
  {
    try
    {
      locked_writer->Join();
      writer.reset();
    }
    catch (const util::tInterruptedException& e)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: TCPConnection::disconnect() - Interrupted waiting for writer thread.");
    }
  }
  std::shared_ptr<tReader> locked_reader = reader.lock();
  if (locked_reader != NULL && util::tThread::CurrentThread() != locked_reader)
  {
    try
    {
      locked_reader->Join();
      reader.reset();
    }
    catch (const util::tInterruptedException& e)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: TCPConnection::disconnect() - Interrupted waiting for reader thread.");
    }
  }
}

int tTCPConnection::GetAvgPingTime()
{
  int result = 0;
  for (size_t i = 0u; i < ping_times.length; i++)
  {
    result += ping_times[i];
  }
  return result / ping_times.length;
}

int tTCPConnection::GetMaxPingTime()
{
  int result = 0;
  for (size_t i = 0u; i < ping_times.length; i++)
  {
    result = std::max(result, ping_times[i]);
  }
  return result;
}

int tTCPConnection::GetRx()
{
  int64 last_time = last_rx_timestamp;
  int64 last_pos = last_rx_position;
  last_rx_timestamp = util::tTime::GetCoarse();
  last_rx_position = cis->GetAbsoluteReadPosition();
  if (last_time == 0)
  {
    return 0;
  }
  if (last_rx_timestamp == last_time)
  {
    return 0;
  }

  double data = last_rx_position - last_pos;
  double interval = (last_rx_timestamp - last_time) / 1000;
  return static_cast<int>((data / interval));
}

void tTCPConnection::HandleMethodCall()
{
  // read port index and retrieve proxy port
  int handle = cis->ReadInt();
  int remote_handle = cis->ReadInt();
  rrlib::serialization::tDataTypeBase method_type = cis->ReadType();
  cis->ReadSkipOffset();
  tTCPPort* port = LookupPortForCallHandling(handle);

  if ((port == NULL || method_type == NULL || (!core::tFinrocTypeInfo::IsMethodType(method_type))))
  {
    // create/decode call
    core::tMethodCall* mc = core::tThreadLocalCache::GetFast()->GetUnusedMethodCall();
    try
    {
      mc->DeserializeCall(cis.get(), method_type, true);
    }
    catch (const util::tException& e)
    {
      mc->Recycle();
      return;
    }

    mc->SetExceptionStatus(core::tMethodCallException::eNO_CONNECTION);
    mc->SetRemotePortHandle(remote_handle);
    mc->SetLocalPortHandle(handle);
    SendCall(mc);
    cis->ToSkipTarget();
  }

  // make sure, "our" port is not deleted while we use it
  {
    util::tLock lock2(port->GetPort());

    bool skip_call = (!port->GetPort()->IsReady());
    core::tMethodCall* mc = core::tThreadLocalCache::GetFast()->GetUnusedMethodCall();
    cis->SetBufferSource(skip_call ? NULL : port->GetPort());
    try
    {
      mc->DeserializeCall(cis.get(), method_type, skip_call);
    }
    catch (const util::tException& e)
    {
      cis->SetBufferSource(NULL);
      mc->Recycle();
      return;
    }
    cis->SetBufferSource(NULL);

    // process call
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_VERBOSE_2, log_domain, "Incoming Server Command: Method call ", (port != NULL ? port->GetPort()->GetQualifiedName() : handle));
    if (skip_call)
    {
      mc->SetExceptionStatus(core::tMethodCallException::eNO_CONNECTION);
      mc->SetRemotePortHandle(remote_handle);
      mc->SetLocalPortHandle(handle);
      SendCall(mc);
      cis->ToSkipTarget();
    }
    else
    {
      core::tNetPort::tInterfaceNetPortImpl* inp = static_cast<core::tNetPort::tInterfaceNetPortImpl*>(port->GetPort());
      inp->ProcessCallFromNet(mc);
    }
  }
}

void tTCPConnection::HandleMethodCallReturn()
{
  // read port index and retrieve proxy port
  int handle = cis->ReadInt();
  /*int remoteHandle =*/
  cis->ReadInt();
  rrlib::serialization::tDataTypeBase method_type = cis->ReadType();
  cis->ReadSkipOffset();
  tTCPPort* port = LookupPortForCallHandling(handle);

  // skip call?
  if (port == NULL)
  {
    cis->ToSkipTarget();
    return;
  }

  // make sure, "our" port is not deleted while we use it
  {
    util::tLock lock2(port->GetPort());

    if (!port->GetPort()->IsReady())
    {
      cis->ToSkipTarget();
      return;
    }

    // create/decode call
    core::tMethodCall* mc = core::tThreadLocalCache::GetFast()->GetUnusedMethodCall();
    //boolean skipCall = (methodType == null || (!methodType.isMethodType()));
    cis->SetBufferSource(port->GetPort());
    try
    {
      mc->DeserializeCall(cis.get(), method_type, false);
    }
    catch (const util::tException& e)
    {
      cis->ToSkipTarget();
      cis->SetBufferSource(NULL);
      mc->Recycle();
      return;
    }
    cis->SetBufferSource(NULL);

    // process call
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_VERBOSE_2, log_domain, "Incoming Server Command: Method call return ", (port != NULL ? port->GetPort()->GetQualifiedName() : handle));

    // process call
    port->HandleCallReturnFromNet(mc);
  }
}

void tTCPConnection::HandlePullCall()
{
  // read port index and retrieve proxy port
  //      int remoteHandle = cis.readInt();
  int handle = cis->ReadInt();
  int remote_handle = cis->ReadInt();
  cis->ReadSkipOffset();
  tTCPPort* port = LookupPortForCallHandling(handle);

  // create/decode call
  core::tPullCall* pc = core::tThreadLocalCache::GetFast()->GetUnusedPullCall();
  try
  {
    pc->Deserialize(*cis);
  }
  catch (const util::tException& e)
  {
    pc->Recycle();
    return;
  }

  // process call
  FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_VERBOSE_2, log_domain, "Incoming Server Command to port '", (port != NULL ? port->GetPort()->GetQualifiedName() : handle), "': ", pc->ToString());

  if (port == NULL || (!port->GetPort()->IsReady()))
  {
    pc->SetExceptionStatus(core::tMethodCallException::eNO_CONNECTION);
    pc->SetRemotePortHandle(remote_handle);
    pc->SetLocalPortHandle(handle);
    SendCall(pc);
  }
  else
  {
    // Execute pull in extra thread, since it can block
    //          pc.setRemotePortHandle(remoteHandle);
    pc->PrepareForExecution(port);
    core::tRPCThreadPool::GetInstance()->ExecuteTask(pc);
  }
}

void tTCPConnection::HandleReturningPullCall()
{
  int handle = cis->ReadInt();
  /*int remoteHandle =*/
  cis->ReadInt();
  cis->ReadSkipOffset();
  tTCPPort* port = LookupPortForCallHandling(handle);

  if (port == NULL || (!port->GetPort()->IsReady()))
  {
    // port does not exist anymore - discard call
    cis->ToSkipTarget();
    return;
  }

  // make sure, "our" port is not deleted while we use it
  {
    util::tLock lock2(port->GetPort());

    // check ready again...
    if (!port->GetPort()->IsReady())
    {
      // port is deleted - discard call
      cis->ToSkipTarget();
      return;
    }

    // deserialize pull call
    core::tPullCall* pc = core::tThreadLocalCache::GetFast()->GetUnusedPullCall();
    try
    {
      cis->SetBufferSource(port->GetPort());
      pc->Deserialize(*cis);
      cis->SetBufferSource(NULL);

      // debug output
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_VERBOSE_2, log_domain, "Incoming Server Command: Pull return call ", (port != NULL ? port->GetPort()->GetQualifiedName() : handle), " status: ", pc->GetStatusString());

    }
    catch (const util::tException& e)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_WARNING, log_domain, e);
      pc->Recycle();
      pc = NULL;
    }

    if (pc != NULL)
    {
      port->HandleCallReturnFromNet(pc);
    }
  }
}

void tTCPConnection::NotifyWriter()
{
  std::shared_ptr<tWriter> locked_writer = writer.lock();
  if (locked_writer != NULL)
  {
    locked_writer->NotifyWriter();
  }
}

bool tTCPConnection::PingTimeExceeed()
{
  std::shared_ptr<tWriter> locked_writer = writer.lock();
  if (locked_writer == NULL)
  {
    return false;
  }
  if (last_acknowledged_packet != locked_writer->cur_packet_index)
  {
    int64 critical_packet_time = sent_packet_time[(last_acknowledged_packet + 1) & tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS];
    int64 time_left = critical_packet_time + tTCPSettings::GetInstance()->critical_ping_threshold.GetValue() - util::tSystem::CurrentTimeMillis();
    return time_left < 0;
  }
  else
  {
    return false;
  }
}

void tTCPConnection::SendCall(core::tSerializableReusable* call)
{
  std::shared_ptr<tWriter> locked_writer = writer.lock();
  if (locked_writer != NULL && (!disconnect_signal))
  {
    locked_writer->SendCall(call);
  }
}

bool tTCPConnection::SendDataPrototype(int64 start_time, int8 op_code)
{
  bool request_acknowledgement = false;

  // send port data
  util::tArrayWrapper<tTCPPort*>* it = monitored_ports.GetIterable();
  int8 changed_flag = 0;
  for (size_t i = 0u, n = monitored_ports.Size(); i < n; i++)
  {
    tTCPPort* pp = it->Get(i);
    if (pp != NULL && pp->GetPort()->IsReady())
    {
      if (pp->GetLastUpdate() + pp->GetUpdateIntervalForNet() > start_time)
      {
        // value cannot be written in this iteration due to minimal update rate
        NotifyWriter();

      }
      else if ((changed_flag = pp->GetPort()->GetChanged()) > core::tAbstractPort::cNO_CHANGE)
      {
        pp->GetPort()->ResetChanged();
        request_acknowledgement = true;
        pp->SetLastUpdate(start_time);

        // execute/write set command to stream
        cos->WriteByte(op_code);
        cos->WriteInt(pp->GetRemoteHandle());
        cos->WriteSkipOffsetPlaceholder();
        cos->WriteByte(changed_flag);
        pp->WriteDataToNetwork(cos.get(), start_time);
        cos->SkipTargetHere();
        TerminateCommand();
      }
    }
  }
  core::tThreadLocalCache::GetFast()->ReleaseAllLocks();
  return request_acknowledgement;
}

void tTCPConnection::TerminateCommand()
{
  if (tTCPSettings::cDEBUG_TCP)
  {
    cos->WriteInt(tTCPSettings::cDEBUG_TCP_NUMBER);
  }
}

void tTCPConnection::UpdateTimeChanged(rrlib::serialization::tDataTypeBase dt, int16 new_update_time)
{
  // forward update time change to connection partner
  tTCPCommand* tc = tTCP::GetUnusedTCPCommand();
  tc->op_code = tTCP::cUPDATETIME;
  tc->datatypeuid = dt == NULL ? -1 : dt.GetUid();
  tc->update_interval = new_update_time;
  SendCall(tc);
}

tTCPConnection::tReader::tReader(tTCPConnection* const outer_class_ptr_, const util::tString& description) :
    core::tCoreLoopThreadBase(-1, false, false),
    outer_class_ptr(outer_class_ptr_)
{
  SetName(description);
  LockObject(outer_class_ptr->socket);
}

void tTCPConnection::tReader::CheckCommandEnd()
{
  if (tTCPSettings::cDEBUG_TCP)
  {
    int i = outer_class_ptr->cis->ReadInt();
    if (i != tTCPSettings::cDEBUG_TCP_NUMBER)
    {
      throw util::tRuntimeException("TCP Stream seems corrupt", CODE_LOCATION_MACRO);
    }
  }
}

void tTCPConnection::tReader::Run()
{
  InitThreadLocalCache();
  // only for c++ automatic deallocation
  ::std::shared_ptr<core::tCoreInput> cis = outer_class_ptr->cis;

  try
  {
    while (!outer_class_ptr->disconnect_signal)
    {
      // we are waiting for change events and acknowledgement related stuff
      int8 op_code = cis->ReadByte();
      //System.out.println("Incoming command - opcode " + opCode);

      // create vars before switch-statement (because of C++)
      int ack_req_index = 0;
      int index = 0;
      int64 cur_time = 0;
      tPeerList* pl = NULL;
      bool notify_writers = false;
      int16 uid = 0;

      // process acknowledgement stuff and other commands common for server and client
      switch (op_code)
      {
      case tTCP::cPING:
        ack_req_index = cis->ReadInt();
        if (ack_req_index != outer_class_ptr->last_ack_request_index + 1)
        {
          throw util::tException("Invalid acknowledgement request");
        }
        outer_class_ptr->last_ack_request_index++;  // not atomic, but doesn't matter here - since this is the only thread that writes
        outer_class_ptr->NotifyWriter();
        break;

      case tTCP::cPONG:
        index = cis->ReadInt();

        // set ping times
        cur_time = util::tSystem::CurrentTimeMillis();
        for (int i = outer_class_ptr->last_acknowledged_packet + 1; i <= index; i++)
        {
          outer_class_ptr->ping_times[i & tTCPSettings::cAVG_PING_PACKETS] = static_cast<int>((cur_time - outer_class_ptr->sent_packet_time[i & tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS]));
        }

        outer_class_ptr->last_acknowledged_packet = index;
        outer_class_ptr->NotifyWriter();
        break;

      case tTCP::cUPDATETIME:
        uid = cis->ReadShort();
        outer_class_ptr->update_times.SetTime(uid, cis->ReadShort());
        break;

      case tTCP::cPULLCALL:
        outer_class_ptr->HandlePullCall();
        break;

      case tTCP::cPULLCALL_RETURN:
        outer_class_ptr->HandleReturningPullCall();
        break;

      case tTCP::cMETHODCALL:
        outer_class_ptr->HandleMethodCall();
        break;

      case tTCP::cMETHODCALL_RETURN:
        outer_class_ptr->HandleMethodCallReturn();
        break;

      case tTCP::cPEER_INFO:
        cis->ReadSkipOffset();
        if (outer_class_ptr->peer == NULL)
        {
          cis->ToSkipTarget();
          break;
        }
        assert((outer_class_ptr->peer != NULL));
        pl = outer_class_ptr->peer->GetPeerList();
        notify_writers = false;
        {
          util::tLock lock5(pl);
          index = pl->GetRevision();
          util::tIPAddress ia = util::tIPAddress::Deserialize(cis.get());
          outer_class_ptr->peer->GetPeerList()->DeserializeAddresses(cis.get(), ia, outer_class_ptr->socket->GetRemoteIPSocketAddress().GetAddress());
          notify_writers = index != pl->GetRevision();
        }
        if (notify_writers)
        {
          outer_class_ptr->peer->NotifyAllWriters();
        }
        break;

      default:
        outer_class_ptr->ProcessRequest(op_code);
        break;
      }
      CheckCommandEnd();
    }
  }
  catch (const util::tException& e)
  {
    if (typeid(e) != typeid(finroc::util::tEOFException) && typeid(e) != typeid(finroc::util::tIOException))
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_WARNING, log_domain, e);
    }

  }
  try
  {
    outer_class_ptr->HandleDisconnect();
  }
  catch (const util::tException& e)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, e);
  }

  try
  {
    cis->Close();
  }
  catch (const util::tException& e)
  {
  }
}

void tTCPConnection::tReader::StopThread()
{
  {
    util::tLock lock2(outer_class_ptr);
    outer_class_ptr->disconnect_signal = true;
    outer_class_ptr->socket->ShutdownReceive();
  }
}

tTCPConnection::tWriter::tWriter(tTCPConnection* const outer_class_ptr_, const util::tString& description) :
    core::tCoreLoopThreadBase(-1, true, false),
    outer_class_ptr(outer_class_ptr_),
    last_ack_index(0),
    cur_packet_index(0),
    handled_changed_counter(-1),
    writer_synch(1u, 30u, 0u, 0u),
    calls_to_send()
{
  SetName(description);
  LockObject(outer_class_ptr->socket);
}

bool tTCPConnection::tWriter::CanSend()
{
  int max_not_ack = outer_class_ptr->max_not_acknowledged_packets.GetValue();
  return cur_packet_index < outer_class_ptr->last_acknowledged_packet + max_not_ack || (max_not_ack <= 0 && cur_packet_index < outer_class_ptr->last_acknowledged_packet + tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS);
}

tTCPConnection::tWriter::~tWriter()
{
  // recycle calls
  core::tSerializableReusable* call = NULL;
  while ((call = calls_to_send.Dequeue()) != NULL)
  {
    assert((call->StateChange(util::tAbstractReusable::cENQUEUED, util::tAbstractReusable::cPOST_QUEUED, outer_class_ptr)));
    //call.responsibleThread = ThreadUtil.getCurrentThreadId();
    call->GenericRecycle();  // recycle complete call
  }
}

void tTCPConnection::tWriter::NotifyWriter()
{
  while (true)
  {
    int raw = writer_synch.GetRaw();
    int sleeping = writer_synch.GetVal1(raw);
    int counter = writer_synch.GetVal2(raw) + 1;
    if (sleeping == 0)    // typical case: increment changed counter by one
    {
      if (writer_synch.CompareAndSet(raw, 0u, counter))
      {
        return;
      }
    }
    else    // writer thread is sleeping...
    {
      if (CanSend())
      {
        {
          util::tLock lock5(this);
          if (writer_synch.CompareAndSet(raw, 0u, counter))
          {
            monitor.Notify(lock5);
            return;
          }
        }
      }
      else
      {
        if (writer_synch.CompareAndSet(raw, 1u, counter))    // thread is still sleeping... but increment counter
        {
          return;
        }
      }
    }
  }
}

void tTCPConnection::tWriter::Run()
{
  InitThreadLocalCache();
  ::std::shared_ptr<core::tCoreOutput> cos = outer_class_ptr->cos;

  try
  {
    while (true)
    {
      // this is "trap" for writer thread... stays in here as long as nothing has changed
      do
      {
        if (outer_class_ptr->disconnect_signal)
        {
          try
          {
            outer_class_ptr->HandleDisconnect();
          }
          catch (const util::tException& e)
          {
            FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, e);
          }
          //cleanShutdown();
          return;
        }

        // has something changed since last iteration? => continue
        int raw = writer_synch.GetRaw();
        assert((writer_synch.GetVal1(raw) == 0));
        int change_count = writer_synch.GetVal2(raw);
        if (change_count != handled_changed_counter)
        {
          handled_changed_counter = change_count;
          break;
        }

        // okay... seems nothing has changed... set synch variable to sleeping
        {
          util::tLock lock5(this);
          try
          {
            if (writer_synch.CompareAndSet(raw, 1u, change_count))
            {
              monitor.Wait(lock5, 10000);  // 10000 to avoid unlucky dead locks while disconnecting
            }
            // if compare and set failed... there was a change => continue
          }
          catch (const util::tInterruptedException& e)
          {
            // continue, because something changed
          }

          // reset changed flag
          while (!outer_class_ptr->disconnect_signal)
          {
            int raw2 = writer_synch.GetRaw();
            if (writer_synch.CompareAndSet(raw2, 0u, writer_synch.GetVal2(raw2)))
            {
              break;
            }
          }
        }
      }
      while (true);

      int64 start_time = util::tSystem::CurrentTimeMillis();

      // send acknowledgements
      SendAcknowledgementsAndCommands();

      // send data
      this->tc->ReleaseAllLocks();
      bool request_acknowledgement = outer_class_ptr->SendData(start_time);
      if (!request_acknowledgement && outer_class_ptr->last_acknowledged_packet == cur_packet_index)
      {
        request_acknowledgement = util::tTime::GetCoarse() > outer_class_ptr->sent_packet_time[outer_class_ptr->last_acknowledged_packet & tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS] + 1000;
        /*if (requestAcknowledgement) {
            System.out.println("requesting ack - because we haven't done that for a long time " + Time.getCoarse());
        }*/
      }

      if (request_acknowledgement)
      {
        cos->WriteByte(tTCP::cPING);
        cur_packet_index++;
        cos->WriteInt(cur_packet_index);
        outer_class_ptr->sent_packet_time[cur_packet_index & tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS] = util::tSystem::CurrentTimeMillis();
        outer_class_ptr->TerminateCommand();
      }

      // send acknowledgements - again
      SendAcknowledgementsAndCommands();

      cos->Flush();
      this->tc->ReleaseAllLocks();

      // wait for minimum update time
      int64 wait_for = (static_cast<int64>(outer_class_ptr->min_update_interval.GetValue())) - (util::tSystem::CurrentTimeMillis() - start_time);
      assert((wait_for <= outer_class_ptr->min_update_interval.GetValue()));
      if ((wait_for > 0) && (!outer_class_ptr->disconnect_signal))
      {
        ::finroc::util::tThread::Sleep(wait_for);
      }
    }
  }
  catch (const util::tException& e)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, e);

    try
    {
      outer_class_ptr->HandleDisconnect();
    }
    catch (const util::tException& e2)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, e2);
    }
  }

  try
  {
    cos->Close();
  }
  catch (const util::tException& e)
  {
  }
}

void tTCPConnection::tWriter::SendAcknowledgementsAndCommands()
{
  // send receive notification
  if (last_ack_index < outer_class_ptr->last_ack_request_index)
  {
    outer_class_ptr->cos->WriteByte(tTCP::cPONG);
    last_ack_index = outer_class_ptr->last_ack_request_index;
    outer_class_ptr->cos->WriteInt(last_ack_index);
    outer_class_ptr->TerminateCommand();
  }

  // send waiting method calls
  core::tSerializableReusable* call = NULL;
  while ((call = calls_to_send.Dequeue()) != NULL)
  {
    assert((call->StateChange(util::tAbstractReusable::cENQUEUED, util::tAbstractReusable::cPOST_QUEUED, outer_class_ptr)));
    if (typeid(*call) == typeid(core::tPullCall))
    {
      core::tPullCall* pc = static_cast<core::tPullCall*>(call);
      outer_class_ptr->cos->WriteByte(pc->IsReturning(true) ? tTCP::cPULLCALL_RETURN : tTCP::cPULLCALL);
      outer_class_ptr->cos->WriteInt(pc->GetRemotePortHandle());
      outer_class_ptr->cos->WriteInt(pc->GetLocalPortHandle());
      outer_class_ptr->cos->WriteSkipOffsetPlaceholder();
    }
    else if (typeid(*call) == typeid(core::tMethodCall))
    {
      core::tMethodCall* mc = static_cast<core::tMethodCall*>(call);
      //assert(mc.getMethod() != null); can be null - if type is not known and we want to return call
      outer_class_ptr->cos->WriteByte(mc->IsReturning(true) ? tTCP::cMETHODCALL_RETURN : tTCP::cMETHODCALL);
      outer_class_ptr->cos->WriteInt(mc->GetRemotePortHandle());
      outer_class_ptr->cos->WriteInt(mc->GetLocalPortHandle());
      outer_class_ptr->cos->WriteType(mc->GetPortInterfaceType());
      outer_class_ptr->cos->WriteSkipOffsetPlaceholder();
    }
    call->Serialize(*outer_class_ptr->cos);
    if (typeid(*call) == typeid(core::tPullCall) || typeid(*call) == typeid(core::tMethodCall))
    {
      outer_class_ptr->cos->SkipTargetHere();
    }
    outer_class_ptr->TerminateCommand();
    call->GenericRecycle();  // call is sent... we do not need any of its objects anymore
  }

  // send updates on connected peers
  if (outer_class_ptr->send_peer_info_to_partner && outer_class_ptr->peer != NULL)
  {
    int peer_info_revision = outer_class_ptr->peer->GetPeerList()->GetRevision();
    if (outer_class_ptr->last_peer_info_sent_revision < peer_info_revision)
    {
      outer_class_ptr->last_peer_info_sent_revision = peer_info_revision;
      outer_class_ptr->cos->WriteByte(tTCP::cPEER_INFO);
      outer_class_ptr->cos->WriteSkipOffsetPlaceholder();
      outer_class_ptr->socket->GetRemoteIPSocketAddress().GetAddress().Serialize(outer_class_ptr->cos.get());
      outer_class_ptr->peer->GetPeerList()->SerializeAddresses(outer_class_ptr->cos.get());
      outer_class_ptr->cos->SkipTargetHere();
      outer_class_ptr->TerminateCommand();
    }
  }
}

void tTCPConnection::tWriter::SendCall(core::tSerializableReusable* call)
{
  //call.responsibleThread = -1;
  assert((call->StateChange(static_cast<int8>((util::tAbstractReusable::cUNKNOWN | util::tAbstractReusable::cUSED)), util::tAbstractReusable::cENQUEUED, outer_class_ptr)));
  calls_to_send.Enqueue(call);
  NotifyWriter();
}

void tTCPConnection::tWriter::StopThread()
{
  util::tLock lock1(this);
  outer_class_ptr->disconnect_signal = true;
  if (writer_synch.GetVal1() != 0)    // if thread is waiting... wake up
  {
    monitor.Notify(lock1);
  }
}

} // namespace finroc
} // namespace tcp

