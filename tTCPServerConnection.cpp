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
#include "plugins/tcp/tTCPServer.h"

#include "plugins/tcp/tTCPServerConnection.h"
#include "rrlib/finroc_core_utils/tAutoDeleter.h"
#include "plugins/tcp/tTCP.h"
#include "rrlib/finroc_core_utils/stream/tLargeIntermediateStreamBuffer.h"
#include "core/tRuntimeEnvironment.h"
#include "core/datatype/tNumber.h"
#include "rrlib/serialization/tDataTypeBase.h"
#include "rrlib/finroc_core_utils/thread/sThreadUtil.h"
#include "core/port/tAbstractPort.h"
#include "core/port/tPortFlags.h"
#include "core/tLockOrderLevels.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "core/datatype/tFrameworkElementInfo.h"
#include "core/port/net/tRemoteTypes.h"
#include "plugins/tcp/tTCPCommand.h"
#include "core/portdatabase/tFinrocTypeInfo.h"
#include "plugins/tcp/tTCPSettings.h"
#include "core/tCoreFlags.h"
#include "core/port/net/tNetPort.h"
#include "rrlib/finroc_core_utils/tTime.h"
#include "core/parameter/tParameterNumeric.h"

namespace finroc
{
namespace tcp
{
util::tSafeConcurrentlyIterableList<tTCPServerConnection*>* tTCPServerConnection::connections = util::tAutoDeleter::AddStatic(new util::tSafeConcurrentlyIterableList<tTCPServerConnection*>(4u, 4u));
util::tAtomicInt tTCPServerConnection::connection_id;

tTCPServerConnection::tTCPServerConnection(std::shared_ptr<util::tNetSocket>& s, int8 stream_id, tTCPServer* server, tTCPPeer* peer) :
    tTCPConnection(stream_id, stream_id == tTCP::cTCP_P2P_ID_BULK ? peer : NULL, stream_id == tTCP::cTCP_P2P_ID_BULK),
    port_set(NULL),
    send_runtime_info(false),
    runtime_info_buffer(false),
    runtime_info_writer(&(runtime_info_buffer)),
    runtime_info_reader(&(runtime_info_buffer.GetDestructiveSource())),
    element_filter(),
    tmp(),
    disconnect_calls(0)
{
  this->socket = s;

  {
    util::tLock lock2(this);

    // initialize core streams (counter part to RemoteServer.Connection constructor)
    std::shared_ptr<util::tLargeIntermediateStreamBuffer> lm_buf(new util::tLargeIntermediateStreamBuffer(s->GetSink()));
    this->cos = std::shared_ptr<rrlib::serialization::tOutputStream>(new rrlib::serialization::tOutputStream(lm_buf, this->update_times));
    //cos = new CoreOutputStream(new BufferedOutputStreamMod(s.getOutputStream()));
    this->cos->WriteLong(core::tRuntimeEnvironment::GetInstance()->GetCreationTime());  // write base timestamp
    //RemoteTypes.serializeLocalDataTypes(cos);
    this->cos->WriteType(core::tNumber::cTYPE);
    this->cos->Flush();

    // init port set here, since it might be serialized to stream
    port_set = new tPortSet(this, server, std::shared_ptr<tTCPServerConnection>(this));
    port_set->Init();

    this->cis = std::shared_ptr<rrlib::serialization::tInputStream>(new rrlib::serialization::tInputStream(s->GetSource(), this->update_times));
    //updateTimes.deserialize(cis);
    cis->SetTimeout(1000);
    rrlib::serialization::tDataTypeBase dt = this->cis->ReadType();
    assert((dt == core::tNumber::cTYPE));

    util::tString type_string = GetConnectionTypeString();

    // send runtime information?
    if (this->cis->ReadBoolean())
    {
      element_filter.Deserialize(*this->cis);
      send_runtime_info = true;
      {
        util::tLock lock4(core::tRuntimeEnvironment::GetInstance()->GetRegistryLock());  // lock runtime so that we do not miss a change
        core::tRuntimeEnvironment::GetInstance()->AddListener(this);

        element_filter.TraverseElementTree(core::tRuntimeEnvironment::GetInstance(), this, false, tmp);
      }
      this->cos->WriteByte(0);  // terminator
      this->cos->Flush();
    }
    cis->SetTimeout(0);

    // start incoming data listener thread
    std::shared_ptr<tTCPConnection::tReader> listener = util::sThreadUtil::GetThreadSharedPtr(new tTCPConnection::tReader(this, util::tStringBuilder("TCP Server ") + type_string + "-Listener for " + s->GetRemoteSocketAddress().ToString()));
    this->reader = listener;
    listener->LockObject(port_set->connection_lock);
    listener->Start();

    // start writer thread
    std::shared_ptr<tTCPConnection::tWriter> writer = util::sThreadUtil::GetThreadSharedPtr(new tTCPConnection::tWriter(this, util::tStringBuilder("TCP Server ") + type_string + "-Writer for " + s->GetRemoteSocketAddress().ToString()));
    this->writer = writer;
    writer->LockObject(port_set->connection_lock);
    writer->Start();

    connections->Add(this, false);
    tPingTimeMonitor::GetInstance();  // start ping time monitor
  }
}

tTCPServerConnection::tServerPort* tTCPServerConnection::GetPort(int handle, bool possibly_create)
{
  core::tAbstractPort* org_port = core::tRuntimeEnvironment::GetInstance()->GetPort(handle);
  if (org_port == NULL)
  {
    return NULL;
  }
  if (org_port->IsChildOf(port_set))
  {
    return static_cast<tServerPort*>(org_port->AsNetPort());
  }
  tServerPort* sp = static_cast<tServerPort*>(org_port->FindNetPort(this));
  if (sp == NULL && possibly_create)
  {
    sp = new tServerPort(this, org_port, port_set);
    sp->GetPort()->Init();
  }
  return sp;
}

void tTCPServerConnection::HandleDisconnect()
{
  // make sure that disconnect is only called once... prevents deadlocks cleaning up all the threads
  int calls = disconnect_calls.IncrementAndGet();
  if (calls > 1)
  {
    return;
  }

  {
    util::tLock lock2(port_set);
    bool port_set_deleted = port_set->IsDeleted();
    {
      util::tLock lock3(this);
      Disconnect();
      if (!port_set_deleted)
      {
        port_set->ManagedDelete();
      }
    }
  }
}

core::tPortCreationInfoBase tTCPServerConnection::InitPci(core::tAbstractPort* counter_part)
{
  core::tPortCreationInfoBase pci(0u);
  pci.max_queue_size = 0;
  pci.parent = port_set;
  int flags = 0;
  if (counter_part->IsOutputPort())
  {
    // create input port
    flags |= core::tPortFlags::cHAS_QUEUE | core::tPortFlags::cACCEPTS_DATA | core::tPortFlags::cUSES_QUEUE;

  }
  else
  {
    // create output io port
    flags |= core::tPortFlags::cIS_OUTPUT_PORT | core::tPortFlags::cMAY_ACCEPT_REVERSE_DATA | core::tPortFlags::cEMITS_DATA;
  }
  pci.flags = flags;
  pci.data_type = counter_part->GetDataType();
  pci.lock_order = core::tLockOrderLevels::cREMOTE_PORT;
  return pci;
}

void tTCPServerConnection::ProcessRequest(int8 op_code)
{
  int handle = 0;
  tServerPort* p = NULL;

  switch (op_code)
  {
  case tTCP::cSET:  // Set data command

    handle = this->cis->ReadInt();
    this->cis->ReadSkipOffset();

    //long timestamp = readTimestamp();
    p = GetPort(handle, true);
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_VERBOSE_2, log_domain, "Incoming Server Command: Set ", (p != NULL ? p->local_port->GetQualifiedName() : handle));
    if (p != NULL)
    {
      {
        util::tLock lock4(p->GetPort());
        if (!p->GetPort()->IsReady())
        {
          this->cis->ToSkipTarget();
        }
        else
        {
          int8 changed_flag = this->cis->ReadByte();
          this->cis->SetFactory(p->GetPort());
          p->ReceiveDataFromStream(*this->cis, util::tSystem::CurrentTimeMillis(), changed_flag);
          this->cis->SetFactory(NULL);
        }
      }
    }
    else
    {
      this->cis->ToSkipTarget();
    }
    break;

  case tTCP::cUNSUBSCRIBE:  // Unsubscribe data

    handle = this->cis->ReadInt();
    p = GetPort(handle, false);
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_VERBOSE_2, log_domain, "Incoming Server Command: Unsubscribe ", (p != NULL ? p->local_port->GetQualifiedName() : handle));
    if (p != NULL && p->GetPort()->IsReady())    // complete disconnect
    {
      p->ManagedDelete();
    }
    break;

  default:
    throw util::tRuntimeException("Unknown OpCode", CODE_LOCATION_MACRO);

  case tTCP::cSUBSCRIBE:  // Subscribe to data

    handle = this->cis->ReadInt();
    int16 strategy = this->cis->ReadShort();
    bool reverse_push = this->cis->ReadBoolean();
    int16 update_interval = this->cis->ReadShort();
    int remote_handle = this->cis->ReadInt();
    p = GetPort(handle, true);
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_VERBOSE_2, log_domain, "Incoming Server Command: Subscribe ", (p != NULL ? p->local_port->GetQualifiedName() : handle), " ", strategy, " ", reverse_push, " ", update_interval, " ", remote_handle);
    if (p != NULL)
    {
      {
        util::tLock lock4(p->GetPort()->GetRegistryLock());
        if (p->GetPort()->IsReady())
        {
          p->GetPort()->SetMinNetUpdateInterval(update_interval);
          p->update_interval_partner = update_interval;
          p->SetRemoteHandle(remote_handle);
          p->GetPort()->SetReversePushStrategy(reverse_push);
          p->PropagateStrategyFromTheNet(strategy);
        }
      }
    }
    break;
  }
}

void tTCPServerConnection::RuntimeChange(int8 change_type, core::tFrameworkElement* element)
{
  if (element != core::tRuntimeEnvironment::GetInstance() && element_filter.Accept(element, tmp, change_type == tRuntimeListener::cREMOVE ? (core::tCoreFlags::cREADY | core::tCoreFlags::cDELETED) : 0) && change_type != ::finroc::core::tRuntimeListener::cPRE_INIT)
  {
    SerializeRuntimeChange(change_type, element);
  }
}

void tTCPServerConnection::RuntimeEdgeChange(int8 change_type, core::tAbstractPort* source, core::tAbstractPort* target)
{
  if (element_filter.Accept(source, tmp) && element_filter.IsAcceptAllFilter())
  {
    SerializeRuntimeChange(core::tFrameworkElementInfo::cEDGE_CHANGE, source);
  }
}

bool tTCPServerConnection::SendData(int64 start_time)
{
  // send port data
  bool request_acknowledgement = ::finroc::tcp::tTCPConnection::SendDataPrototype(start_time, tTCP::cCHANGE_EVENT);

  // updated runtime information
  while (runtime_info_reader.MoreDataAvailable())
  {
    if (this->update_times->TypeUpdateNecessary())
    {
      // use update time tcp command to trigger type update
      tTCPCommand tc;
      tc.op_code = tTCP::cUPDATETIME;
      tc.datatype = core::tNumber::cTYPE;
      tc.update_interval = core::tFinrocTypeInfo::Get(core::tNumber::cTYPE).GetUpdateTime();
      tc.Serialize(*cos);
      TerminateCommand();
    }
    this->cos->WriteAllAvailable(&(runtime_info_reader));
  }

  return request_acknowledgement;
}

void tTCPServerConnection::SerializeRuntimeChange(int8 change_type, core::tFrameworkElement* element)
{
  runtime_info_writer.WriteByte(tTCP::cPORT_UPDATE);
  core::tFrameworkElementInfo::SerializeFrameworkElement(element, change_type, runtime_info_writer, element_filter, tmp);
  if (tTCPSettings::cDEBUG_TCP)
  {
    runtime_info_writer.WriteInt(tTCPSettings::cDEBUG_TCP_NUMBER);
  }
  runtime_info_writer.Flush();
  NotifyWriter();
}

void tTCPServerConnection::TreeFilterCallback(core::tFrameworkElement* fe, bool unused)
{
  if (fe != core::tRuntimeEnvironment::GetInstance())
  {
    if (!fe->IsDeleted())
    {
      this->cos->WriteByte(tTCP::cPORT_UPDATE);
      core::tFrameworkElementInfo::SerializeFrameworkElement(fe, ::finroc::core::tRuntimeListener::cADD, *this->cos, element_filter, tmp);
    }
  }
}

tTCPServerConnection::tPortSet::tPortSet(tTCPServerConnection* const outer_class_ptr_, tTCPServer* server, std::shared_ptr<tTCPServerConnection> connection_lock_) :
    core::tFrameworkElement(server, util::tStringBuilder("connection") + tTCPServerConnection::connection_id.GetAndIncrement(), core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT, core::tLockOrderLevels::cPORT - 1),
    outer_class_ptr(outer_class_ptr_),
    port_iterator(this),
    connection_lock(connection_lock_)
{
}

void tTCPServerConnection::tPortSet::NotifyPortsOfDisconnect()
{
  port_iterator.Reset();
  for (::finroc::core::tFrameworkElement* port = port_iterator.Next(); port != NULL; port = port_iterator.Next())
  {
    (static_cast<core::tAbstractPort*>(port))->NotifyDisconnect();
  }
}

void tTCPServerConnection::tPortSet::PrepareDelete()
{
  outer_class_ptr->HandleDisconnect();
  if (outer_class_ptr->send_runtime_info)
  {
    core::tRuntimeEnvironment::GetInstance()->RemoveListener(outer_class_ptr);
  }
  NotifyPortsOfDisconnect();
  tTCPServerConnection::connections->Remove(outer_class_ptr);
  ::finroc::core::tFrameworkElement::PrepareDelete();
}

tTCPServerConnection::tServerPort::tServerPort(tTCPServerConnection* const outer_class_ptr_, core::tAbstractPort* counter_part, tTCPServerConnection::tPortSet* port_set) :
    tTCPPort(outer_class_ptr_->InitPci(counter_part), outer_class_ptr_->port_set->connection_lock.get()),
    outer_class_ptr(outer_class_ptr_),
    local_port(counter_part)
{
}

void tTCPServerConnection::tServerPort::NotifyDisconnect()
{
  if (local_port->IsInputPort())
  {
    local_port->NotifyDisconnect();
  }
}

void tTCPServerConnection::tServerPort::PostChildInit()
{
  ::finroc::core::tNetPort::PostChildInit();

  // add edge
  if (GetPort()->IsOutputPort())
  {
    GetPort()->ConnectToTarget(local_port);
  }
  else
  {
    GetPort()->ConnectToSource(local_port);
  }
}

std::shared_ptr<tTCPServerConnection::tPingTimeMonitor> tTCPServerConnection::tPingTimeMonitor::instance;
util::tMutexLockOrder tTCPServerConnection::tPingTimeMonitor::static_class_mutex(core::tLockOrderLevels::cINNER_MOST - 20);

tTCPServerConnection::tPingTimeMonitor::tPingTimeMonitor() :
    core::tCoreLoopThreadBase(tTCPSettings::cCONNECTOR_THREAD_LOOP_INTERVAL, false, false)
{
  SetName("TCP Server Ping Time Monitor");
}

tTCPServerConnection::tPingTimeMonitor* tTCPServerConnection::tPingTimeMonitor::GetInstance()
{
  util::tLock lock1(static_class_mutex);
  if (instance.get() == NULL)
  {
    instance = util::sThreadUtil::GetThreadSharedPtr(new tTCPServerConnection::tPingTimeMonitor());
    instance->Start();
  }
  return instance.get();
}

void tTCPServerConnection::tPingTimeMonitor::MainLoopCallback()
{
  int64 start_time = util::tTime::GetCoarse();
  int64 may_wait = tTCPSettings::GetInstance()->critical_ping_threshold.GetValue();

  util::tArrayWrapper<tTCPServerConnection*>* it = connections->GetIterable();
  for (int i = 0, n = connections->Size(); i < n; i++)
  {
    tTCPServerConnection* tsc = it->Get(i);
    if (tsc != NULL)
    {
      may_wait = std::min(may_wait, tsc->CheckPingForDisconnect());  // safe, since connection is deleted deferred and call time is minimal
    }
  }

  // wait remaining uncritical time
  int64 wait_for = may_wait - (util::tTime::GetCoarse() - start_time);
  if (wait_for > 0)
  {
    ::finroc::util::tThread::Sleep(wait_for);
  }
}

} // namespace finroc
} // namespace tcp

