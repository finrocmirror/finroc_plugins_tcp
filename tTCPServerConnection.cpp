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
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "rrlib/finroc_core_utils/stream/tLargeIntermediateStreamBuffer.h"
#include "rrlib/rtti/tDataTypeBase.h"
#include "rrlib/design_patterns/singleton.h"
#include <boost/lexical_cast.hpp>

#include "core/tRuntimeEnvironment.h"
#include "core/datatype/tNumber.h"
#include "core/port/tAbstractPort.h"
#include "core/port/tPortFlags.h"
#include "core/tLockOrderLevels.h"
#include "core/datatype/tFrameworkElementInfo.h"
#include "core/port/net/tRemoteTypes.h"
#include "core/portdatabase/tFinrocTypeInfo.h"
#include "core/tCoreFlags.h"
#include "core/port/net/tNetPort.h"
#include "core/parameter/tParameterNumeric.h"

#include "plugins/tcp/tTCPServerConnection.h"
#include "plugins/tcp/tTCPServer.h"
#include "plugins/tcp/tTCP.h"
#include "plugins/tcp/tTCPCommand.h"
#include "plugins/tcp/tTCPSettings.h"

namespace finroc
{
namespace tcp
{

namespace internal
{
template <typename T>
struct CreateServerConnectionList
{
  static T* Create()
  {
    return new T(4u);
  }
  static void Destroy(T* object)
  {
    delete object;
  }
};

/*!
 * Monitors connections for critical ping time exceed
 */
class tPingTimeMonitor : public core::tCoreLoopThreadBase
{
public:

  tPingTimeMonitor() :
    core::tCoreLoopThreadBase(tTCPSettings::cCONNECTOR_THREAD_LOOP_INTERVAL, false, false)
  {
    SetName("TCP Server Ping Time Monitor");
    Start();
  }

  virtual ~tPingTimeMonitor()
  {
    StopThread();
    Join();
  }

  static tPingTimeMonitor& GetInstance();

  virtual void MainLoopCallback();
};

}

typedef rrlib::design_patterns::tSingletonHolder<util::tSafeConcurrentlyIterableList<tTCPServerConnection*, rrlib::thread::tOrderedMutex>, rrlib::design_patterns::singleton::Longevity, internal::CreateServerConnectionList> tServerConnectionList;
static inline unsigned int GetLongevity(util::tSafeConcurrentlyIterableList<tTCPServerConnection*, rrlib::thread::tOrderedMutex>*)
{
  return 100; // runtime will already be deleted
}

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
  try
  {
    tLock lock2(*this);

    // initialize core streams (counter part to RemoteServer.Connection constructor)
    std::shared_ptr<util::tLargeIntermediateStreamBuffer> lm_buf(new util::tLargeIntermediateStreamBuffer(s->GetSink()));
    this->cos = std::shared_ptr<rrlib::serialization::tOutputStream>(new rrlib::serialization::tOutputStream(lm_buf, this->update_times));
    //cos = new CoreOutputStream(new BufferedOutputStreamMod(s.getOutputStream()));
    (*this->cos) << finroc::core::tRuntimeEnvironment::GetInstance()->GetCreationTime();  // write base timestamp
    //RemoteTypes.serializeLocalDataTypes(cos);
    (*this->cos) << core::tNumber::cTYPE;
    this->cos->Flush();

    // init port set here, since it might be serialized to stream
    port_set = new tPortSet(*this, server, std::shared_ptr<tTCPServerConnection>(this));
    port_set->Init();

    this->cis = std::shared_ptr<rrlib::serialization::tInputStream>(new rrlib::serialization::tInputStream(s->GetSource(), this->update_times));
    //updateTimes.deserialize(cis);
    cis->SetTimeout(std::chrono::seconds(1));
    rrlib::rtti::tDataTypeBase dt;
    (*this->cis) >> dt;
    assert((dt == core::tNumber::cTYPE));

    util::tString type_string = GetConnectionTypeString();

    // send runtime information?
    if (this->cis->ReadBoolean())
    {
      element_filter.Deserialize(*this->cis);
      send_runtime_info = true;
      {
        tLock lock4(core::tRuntimeEnvironment::GetInstance()->GetRegistryLock());  // lock runtime so that we do not miss a change
        core::tRuntimeEnvironment::GetInstance()->AddListener(*this);

        element_filter.TraverseElementTree(*core::tRuntimeEnvironment::GetInstance(), tmp, [&](core::tFrameworkElement & fe)
        {
          if (&fe != core::tRuntimeEnvironment::GetInstance())
          {
            if (!fe.IsDeleted())
            {
              this->cos->WriteEnum(tOpCode::STRUCTURE_UPDATE);
              core::tFrameworkElementInfo::SerializeFrameworkElement(fe, core::tRuntimeListener::cADD, *this->cos, element_filter, tmp);
            }
          }
        });
      }
      this->cos->WriteByte(0);  // terminator
      this->cos->Flush();
    }
    cis->SetTimeout(rrlib::time::tDuration::zero());

    // start incoming data listener thread
    std::shared_ptr<tTCPConnection::tReader> listener = std::static_pointer_cast<tTCPConnection::tReader>((new tTCPConnection::tReader(*this, std::string("TCP Server ") + type_string + "-Listener for " + s->GetRemoteSocketAddress()))->GetSharedPtr());
    this->reader = listener;
    listener->LockObject(port_set->connection_lock);
    listener->Start();
    listener->SetAutoDelete();

    // start writer thread
    std::shared_ptr<tTCPConnection::tWriter> writer = std::static_pointer_cast<tTCPConnection::tWriter>((new tTCPConnection::tWriter(*this, std::string("TCP Server ") + type_string + "-Writer for " + s->GetRemoteSocketAddress()))->GetSharedPtr());
    this->writer = writer;
    writer->LockObject(port_set->connection_lock);
    writer->Start();
    writer->SetAutoDelete();

    tServerConnectionList::Instance().Add(this, false);
    internal::tPingTimeMonitor::GetInstance();  // start ping time monitor
  }
  catch (const std::exception& e)
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_WARNING, e);
    if (port_set)
    {
      port_set->ManagedDelete();
    }
  }
}

tTCPServerConnection::tServerPort* tTCPServerConnection::GetPort(int handle, bool possibly_create)
{
  core::tAbstractPort* org_port = core::tRuntimeEnvironment::GetInstance()->GetPort(handle);
  if (org_port == NULL)
  {
    return NULL;
  }
  if (org_port->IsChildOf(*port_set))
  {
    return static_cast<tServerPort*>(org_port->AsNetPort());
  }
  tServerPort* sp = static_cast<tServerPort*>(core::tNetPort::FindNetPort(*org_port, this));
  if (sp == NULL && possibly_create)
  {
    sp = new tServerPort(*this, org_port, port_set);
    sp->GetPort().Init();
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
    tLock lock2(*port_set);
    bool port_set_deleted = port_set->IsDeleted();

    tLock lock3(*this);
    Disconnect();
    if (!port_set_deleted)
    {
      port_set->ManagedDelete();
    }
  }
}

core::tPortCreationInfoBase tTCPServerConnection::InitPci(core::tAbstractPort* counter_part)
{
  core::tPortCreationInfoBase pci("Port", 0u);
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

void tTCPServerConnection::ProcessRequest(tOpCode op_code)
{
  int handle = 0;
  tServerPort* p = NULL;

  switch (op_code)
  {
  case tOpCode::SET:  // Set data command

    handle = this->cis->ReadInt();
    this->cis->ReadSkipOffset();

    //long timestamp = readTimestamp();
    p = GetPort(handle, true);
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_VERBOSE_2, "Incoming Server Command: Set ", (p != NULL ? p->local_port->GetQualifiedName() : boost::lexical_cast<util::tString>(handle)));
    if (p != NULL)
    {
      {
        tLock lock4(p->GetPort());
        if (!p->GetPort().IsReady())
        {
          this->cis->ToSkipTarget();
        }
        else
        {
          int8 changed_flag = this->cis->ReadByte();
          this->cis->SetFactory(p);
          p->ReceiveDataFromStream(*this->cis, rrlib::time::Now(), changed_flag);
          this->cis->SetFactory(NULL);
        }
      }
    }
    else
    {
      this->cis->ToSkipTarget();
    }
    break;

  case tOpCode::UNSUBSCRIBE:  // Unsubscribe data

    handle = this->cis->ReadInt();
    p = GetPort(handle, false);
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_VERBOSE_2, "Incoming Server Command: Unsubscribe ", (p != NULL ? p->local_port->GetQualifiedName() : boost::lexical_cast<util::tString>(handle)));
    if (p != NULL && p->GetPort().IsReady())    // complete disconnect
    {
      p->ManagedDelete();
    }
    break;

  default:
    throw util::tRuntimeException("Unknown OpCode", CODE_LOCATION_MACRO);

  case tOpCode::SUBSCRIBE:  // Subscribe to data

    handle = this->cis->ReadInt();
    int16 strategy = this->cis->ReadShort();
    bool reverse_push = this->cis->ReadBoolean();
    int16 update_interval = this->cis->ReadShort();
    int remote_handle = this->cis->ReadInt();
    rrlib::serialization::tDataEncoding enc = this->cis->ReadEnum<rrlib::serialization::tDataEncoding>();
    p = GetPort(handle, true);
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_VERBOSE_2, "Incoming Server Command: Subscribe ", (p != NULL ? p->local_port->GetQualifiedName() : boost::lexical_cast<util::tString>(handle)), " ", strategy, " ", reverse_push, " ", update_interval, " ", remote_handle);
    if (p != NULL)
    {
      tLock lock4(p->GetPort().GetRegistryLock());
      if (p->GetPort().IsReady())
      {
        p->SetEncoding(enc);
        p->GetPort().SetMinNetUpdateInterval(update_interval);
        p->update_interval_partner = update_interval;
        p->SetRemoteHandle(remote_handle);
        p->GetPort().SetReversePushStrategy(reverse_push);
        p->PropagateStrategyFromTheNet(strategy);
      }
    }
    break;
  }
}

void tTCPServerConnection::RuntimeChange(int8 change_type, core::tFrameworkElement& element)
{
  if (&element != core::tRuntimeEnvironment::GetInstance() && element_filter.Accept(element, tmp, change_type == tRuntimeListener::cREMOVE ? (core::tCoreFlags::cREADY | core::tCoreFlags::cDELETED) : 0) && change_type != ::finroc::core::tRuntimeListener::cPRE_INIT)
  {
    SerializeRuntimeChange(change_type, element);
  }
}

void tTCPServerConnection::RuntimeEdgeChange(int8 change_type, core::tAbstractPort& source, core::tAbstractPort& target)
{
  if (element_filter.Accept(source, tmp) && element_filter.IsAcceptAllFilter())
  {
    SerializeRuntimeChange(core::tFrameworkElementInfo::cEDGE_CHANGE, source);
  }
}

bool tTCPServerConnection::SendData(const rrlib::time::tTimestamp& start_time)
{
  // send port data
  bool request_acknowledgement = tTCPConnection::SendDataPrototype(start_time, tOpCode::CHANGE_EVENT);

  // updated runtime information
  while (runtime_info_reader.MoreDataAvailable())
  {
    if (this->update_times->TypeUpdateNecessary())
    {
      // use update time tcp command to trigger type update
      tTCPCommand tc;
      tc.op_code = tOpCode::UPDATE_TIME;
      tc.datatype = core::tNumber::cTYPE;
      tc.update_interval = core::tFinrocTypeInfo::Get(core::tNumber::cTYPE).GetUpdateTime();
      tc.Serialize(*cos);
      TerminateCommand();
    }
    this->cos->WriteAllAvailable(&(runtime_info_reader));
  }

  return request_acknowledgement;
}

void tTCPServerConnection::SerializeRuntimeChange(int8 change_type, core::tFrameworkElement& element)
{
  runtime_info_writer.WriteEnum(tOpCode::STRUCTURE_UPDATE);
  core::tFrameworkElementInfo::SerializeFrameworkElement(element, change_type, runtime_info_writer, element_filter, tmp);
  if (tTCPSettings::cDEBUG_TCP)
  {
    runtime_info_writer.WriteInt(tTCPSettings::cDEBUG_TCP_NUMBER);
  }
  runtime_info_writer.Flush();
  NotifyWriter();
}

tTCPServerConnection::tPortSet::tPortSet(tTCPServerConnection& outer_class, tTCPServer* server, std::shared_ptr<tTCPServerConnection> connection_lock_) :
  core::tFrameworkElement(server, std::string("connection") + boost::lexical_cast<util::tString>(tTCPServerConnection::connection_id.GetAndIncrement()), core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT | core::tCoreFlags::cAUTO_RENAME, core::tLockOrderLevels::cPORT - 1),
  outer_class(outer_class),
  port_iterator(*this),
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
  outer_class.HandleDisconnect();
  if (outer_class.send_runtime_info)
  {
    core::tRuntimeEnvironment::GetInstance()->RemoveListener(outer_class);
  }
  NotifyPortsOfDisconnect();
  tServerConnectionList::Instance().Remove(&outer_class);
  tFrameworkElement::PrepareDelete();
}

tTCPServerConnection::tServerPort::tServerPort(tTCPServerConnection& outer_class, core::tAbstractPort* counter_part, tTCPServerConnection::tPortSet* port_set) :
  tTCPPort(outer_class.InitPci(counter_part), outer_class.port_set->connection_lock.get()),
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
  if (GetPort().IsOutputPort())
  {
    GetPort().ConnectToTarget(*local_port);
  }
  else
  {
    GetPort().ConnectToSource(*local_port);
  }
}

namespace internal
{

typedef rrlib::design_patterns::tSingletonHolder<tPingTimeMonitor, rrlib::design_patterns::singleton::Longevity> tPingTimeMonitorInstance;
static inline unsigned int GetLongevity(tPingTimeMonitor*)
{
  return 0; // delete before runtime
}

tPingTimeMonitor& tPingTimeMonitor::GetInstance()
{
  /*! Locked before thread list (in C++) */
  static rrlib::thread::tOrderedMutex static_class_mutex("Ping Time Monitor Instance", core::tLockOrderLevels::cINNER_MOST - 20);

  tLock lock1(static_class_mutex);
  return tPingTimeMonitorInstance::Instance();
}

void tPingTimeMonitor::MainLoopCallback()
{
  rrlib::time::tTimestamp start_time = rrlib::time::Now(false);
  rrlib::time::tDuration may_wait = tTCPSettings::GetInstance()->critical_ping_threshold.Get();

  util::tArrayWrapper<tTCPServerConnection*>* it = tServerConnectionList::Instance().GetIterable();
  for (int i = 0, n = tServerConnectionList::Instance().Size(); i < n; i++)
  {
    tTCPServerConnection* tsc = it->Get(i);
    if (tsc != NULL)
    {
      may_wait = std::min(may_wait, tsc->CheckPingForDisconnect());  // safe, since connection is deleted deferred and call time is minimal
    }
  }

  // wait remaining uncritical time
  rrlib::time::tDuration wait_for = may_wait - (rrlib::time::Now(false) - start_time);
  if (wait_for > rrlib::time::tDuration::zero())
  {
    Sleep(wait_for, false);
  }
}

} // namespace internal

} // namespace tcp
} // namespace finroc

