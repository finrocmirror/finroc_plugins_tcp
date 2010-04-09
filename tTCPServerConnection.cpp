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
#include "tcp/tTCPServerConnection.h"
#include "tcp/tTCP.h"
#include "finroc_core_utils/stream/tLargeIntermediateStreamBuffer.h"
#include "core/tRuntimeEnvironment.h"
#include "core/port/net/tRemoteTypes.h"
#include "core/portdatabase/tDataTypeRegister.h"
#include "finroc_core_utils/thread/sThreadUtil.h"
#include "core/port/tPortFlags.h"
#include "tcp/tTCPSettings.h"
#include "core/datatype/tFrameworkElementInfo.h"
#include "core/tCoreFlags.h"
#include "finroc_core_utils/tTime.h"

namespace finroc
{
namespace tcp
{
util::tSafeConcurrentlyIterableList<tTCPServerConnection*> tTCPServerConnection::connections(4u, 4u);
util::tAtomicInt tTCPServerConnection::connection_id;

tTCPServerConnection::tTCPServerConnection(::std::tr1::shared_ptr<util::tNetSocket> s, int8 stream_id, tTCPServer* server, tTCPPeer* peer) :
    tTCPConnection(stream_id, stream_id == tTCP::cTCP_P2P_ID_BULK ? peer : NULL, stream_id == tTCP::cTCP_P2P_ID_BULK),
    port_set(new tPortSet(this, server, ::std::tr1::shared_ptr<tTCPServerConnection>(this))),
    send_runtime_info(false),
    runtime_info_buffer(false),
    runtime_info_writer(&(runtime_info_buffer)),
    runtime_info_reader(&(runtime_info_buffer.GetDestructiveSource())),
    element_filter(),
    tmp()
{
  this->socket = s;

  // initialize core streams (counter part to RemoteServer.Connection constructor)
  ::std::tr1::shared_ptr<util::tLargeIntermediateStreamBuffer> lm_buf(new util::tLargeIntermediateStreamBuffer(s->GetSink()));
  this->cos = ::std::tr1::shared_ptr<core::tCoreOutput>(new core::tCoreOutput(lm_buf));
  //cos = new CoreOutputStream(new BufferedOutputStreamMod(s.getOutputStream()));
  this->cos->WriteLong(core::tRuntimeEnvironment::GetInstance()->GetCreationTime());  // write base timestamp
  core::tRemoteTypes::SerializeLocalDataTypes(core::tDataTypeRegister::GetInstance(), this->cos.get());
  this->cos->Flush();
  port_set->Init();

  this->cis = ::std::tr1::shared_ptr<core::tCoreInput>(new core::tCoreInput(s->GetSource()));
  this->cis->SetTypeTranslation(&(this->update_times));
  this->update_times.Deserialize(this->cis.get());

  util::tString type_string = GetConnectionTypeString();

  // send runtime information?
  if (this->cis->ReadBoolean())
  {
    element_filter.Deserialize(*this->cis);
    send_runtime_info = true;
    {
      util::tLock lock3(core::tRuntimeEnvironment::GetInstance()->obj_synch);
      core::tRuntimeEnvironment::GetInstance()->AddListener(this);
      element_filter.TraverseElementTree(core::tRuntimeEnvironment::GetInstance(), this, tmp);
    }
    this->cos->WriteByte(0);  // terminator
    this->cos->Flush();
  }

  // start incoming data listener thread
  ::std::tr1::shared_ptr<tTCPConnection::tReader> listener = util::sThreadUtil::GetThreadSharedPtr(new tTCPConnection::tReader(this, util::tStringBuilder("TCP Server ") + type_string + "-Listener for " + s->GetRemoteSocketAddress().ToString()));
  listener->LockObject(port_set->connection_lock);
  listener->Start();

  // start writer thread
  ::std::tr1::shared_ptr<tTCPConnection::tWriter> writer = util::sThreadUtil::GetThreadSharedPtr(new tTCPConnection::tWriter(this, util::tStringBuilder("TCP Server ") + type_string + "-Writer for " + s->GetRemoteSocketAddress().ToString()));
  this->writer = writer;
  writer->LockObject(port_set->connection_lock);
  writer->Start();

  connections.Add(this, false);
  tPingTimeMonitor::GetInstance();  // start ping time monitor
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
  util::tLock lock1(obj_synch);
  Disconnect();
  {
    util::tLock lock2(port_set->obj_synch);
    if (!port_set->IsDeleted())
    {
      port_set->ManagedDelete();
    }
  }
}

core::tPortCreationInfo tTCPServerConnection::InitPci(core::tAbstractPort* counter_part)
{
  core::tPortCreationInfo pci(0);
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
  return pci;
}

void tTCPServerConnection::ProcessRequest(int8 op_code)
{
  int handle = 0;
  tServerPort* p = NULL;

  switch (op_code)
  {
    //    case TCP.PULLCALL: // Pull data request - server side
    //
    //
    //
    //      handle = cis.readInt();
    //      cis.readSkipOffset();
    //      p = getPort(handle, true);
    //
    //      handlePullCall(p, handle, portSet);
    //      break;

    //    case TCP.METHODCALL:
    //
    //      handle = cis.readInt();
    //      cis.readSkipOffset();
    //      p = getPort(handle, true);
    //
    //      // okay... this is handled asynchronously now
    //      // create/decode call
    //      // okay... this is handled asynchronously now
    //      // create/decode call
    //      cis.setBufferSource(p.getPort());
    //      // lookup method type
    //      DataType methodType = cis.readType();
    //      if (methodType == null || (!methodType.isMethodType())) {
    //        cis.toSkipTarget();
    //      } else {
    //        mc = ThreadLocalCache.getFast().getUnusedMethodCall();
    //        mc.deserializeCall(cis, methodType);
    //
    //        // process call
    //        if (TCPSettings.DISPLAY_INCOMING_TCP_SERVER_COMMANDS.get()) {
    //          System.out.println("Incoming Server Command: Method call " + (p != null ? p.getPort().getQualifiedName() : handle));
    //        }
    //        if (p == null) {
    //          mc.setExceptionStatus(MethodCallException.Type.NO_CONNECTION);
    //          sendCall(mc);
    //        } else {
    //          NetPort.InterfaceNetPortImpl inp = (NetPort.InterfaceNetPortImpl)p.getPort();
    //          inp.processCallFromNet(mc);
    //        }
    //      }
    //      cis.setBufferSource(null);
    //
    //      break;

  case tTCP::cSET:  // Set data command

    handle = this->cis->ReadInt();
    this->cis->ReadSkipOffset();

    //long timestamp = readTimestamp();
    p = GetPort(handle, true);
    if (tTCPSettings::cDISPLAY_INCOMING_TCP_SERVER_COMMANDS->Get())
    {
      util::tSystem::out.Println(util::tStringBuilder("Incoming Server Command: Set ") + (p != NULL ? p->local_port->GetQualifiedName() : handle));
    }
    if (p != NULL)
    {
      int8 changed_flag = this->cis->ReadByte();
      this->cis->SetBufferSource(p->GetPort());
      p->ReceiveDataFromStream(this->cis.get(), util::tSystem::CurrentTimeMillis(), changed_flag);
      this->cis->SetBufferSource(NULL);
    }
    else
    {
      this->cis->ToSkipTarget();
    }
    break;

  case tTCP::cUNSUBSCRIBE:  // Unsubscribe data

    handle = this->cis->ReadInt();
    p = GetPort(handle, false);
    if (tTCPSettings::cDISPLAY_INCOMING_TCP_SERVER_COMMANDS->Get())
    {
      util::tSystem::out.Println(util::tStringBuilder("Incoming Server Command: Unsubscribe ") + (p != NULL ? p->local_port->GetQualifiedName() : handle));
    }
    if (p != NULL)    // complete disconnect
    {
      p->ManagedDelete();
    }
    break;

  default:
    throw util::tRuntimeException("Unknown OpCode");

  case tTCP::cSUBSCRIBE:  // Subscribe to data

    handle = this->cis->ReadInt();
    int16 strategy = this->cis->ReadShort();
    bool reverse_push = this->cis->ReadBoolean();
    int16 update_interval = this->cis->ReadShort();
    int remote_handle = this->cis->ReadInt();
    p = GetPort(handle, true);
    if (tTCPSettings::cDISPLAY_INCOMING_TCP_SERVER_COMMANDS->Get())
    {
      util::tSystem::out.Println(util::tStringBuilder("Incoming Server Command: Subscribe ") + (p != NULL ? p->local_port->GetQualifiedName() : handle) + " " + strategy + " " + reverse_push + " " + update_interval + " " + remote_handle);
    }
    if (p != NULL)
    {
      p->GetPort()->SetMinNetUpdateInterval(update_interval);
      p->update_interval_partner = update_interval;
      p->SetRemoteHandle(remote_handle);
      p->GetPort()->SetReversePushStrategy(reverse_push);
      p->PropagateStrategyFromTheNet(strategy);
    }
    break;
  }
}

void tTCPServerConnection::RuntimeChange(int8 change_type, core::tFrameworkElement* element)
{
  if (element != core::tRuntimeEnvironment::GetInstance() && element_filter.Accept(element, tmp) && change_type != ::finroc::core::tRuntimeListener::cPRE_INIT)
  {
    runtime_info_writer.WriteByte(tTCP::cPORT_UPDATE);
    core::tFrameworkElementInfo::SerializeFrameworkElement(element, change_type, &(runtime_info_writer), element_filter, tmp);
    if (tTCPSettings::cDEBUG_TCP)
    {
      runtime_info_writer.WriteInt(tTCPSettings::cDEBUG_TCP_NUMBER);
    }
    runtime_info_writer.Flush();
    NotifyWriter();
  }
}

bool tTCPServerConnection::SendData(int64 start_time)
{
  // send port data
  bool request_acknowledgement = ::finroc::tcp::tTCPConnection::SendDataPrototype(start_time, tTCP::cCHANGE_EVENT);

  // updated runtime information
  while (runtime_info_reader.MoreDataAvailable())
  {
    this->cos->WriteAllAvailable(&(runtime_info_reader));
  }

  return request_acknowledgement;
}

void tTCPServerConnection::TreeFilterCallback(core::tFrameworkElement* fe)
{
  if (fe != core::tRuntimeEnvironment::GetInstance())
  {
    this->cos->WriteByte(tTCP::cPORT_UPDATE);
    core::tFrameworkElementInfo::SerializeFrameworkElement(fe, ::finroc::core::tRuntimeListener::cADD, this->cos.get(), element_filter, tmp);
  }
}

tTCPServerConnection::tPortSet::tPortSet(tTCPServerConnection* const outer_class_ptr_, tTCPServer* server, ::std::tr1::shared_ptr<tTCPServerConnection> connection_lock_) :
    core::tFrameworkElement(util::tStringBuilder("connection") + tTCPServerConnection::connection_id.GetAndIncrement(), server, core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT),
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
  if (outer_class_ptr->send_runtime_info)
  {
    core::tRuntimeEnvironment::GetInstance()->RemoveListener(outer_class_ptr);
  }
  NotifyPortsOfDisconnect();
  tTCPServerConnection::connections.Remove(outer_class_ptr);
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

util::tMutex tTCPServerConnection::tPingTimeMonitor::static_obj_synch;
::std::tr1::shared_ptr<tTCPServerConnection::tPingTimeMonitor> tTCPServerConnection::tPingTimeMonitor::instance;

tTCPServerConnection::tPingTimeMonitor::tPingTimeMonitor() :
    core::tCoreLoopThreadBase(tTCPSettings::cCONNECTOR_THREAD_LOOP_INTERVAL, false, false)
{
  SetName("TCP Server Ping Time Monitor");
}

tTCPServerConnection::tPingTimeMonitor* tTCPServerConnection::tPingTimeMonitor::GetInstance()
{
  util::tLock lock1(static_obj_synch);
  if (instance == NULL)
  {
    instance = util::sThreadUtil::GetThreadSharedPtr(new tTCPServerConnection::tPingTimeMonitor());
    instance->Start();
  }
  return instance.get();
}

void tTCPServerConnection::tPingTimeMonitor::MainLoopCallback()
{
  int64 start_time = util::tTime::GetCoarse();
  int64 may_wait = tTCPSettings::critical_ping_threshold->Get();

  util::tArrayWrapper<tTCPServerConnection*>* it = connections.GetIterable();
  for (int i = 0, n = connections.Size(); i < n; i++)
  {
    tTCPServerConnection* tsc = it->Get(i);
    if (tsc != NULL)
    {
      may_wait = util::tMath::Min(may_wait, tsc->CheckPingForDisconnect());
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

