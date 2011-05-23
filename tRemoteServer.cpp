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
#include "rrlib/finroc_core_utils/tGarbageCollector.h"

#include "plugins/tcp/tRemoteServer.h"
#include "core/tCoreFlags.h"
#include "core/tLockOrderLevels.h"
#include "core/tRuntimeEnvironment.h"
#include "rrlib/finroc_core_utils/thread/sThreadUtil.h"
#include "plugins/tcp/tTCP.h"
#include "plugins/tcp/tTCPPeer.h"
#include "core/port/tPortFlags.h"
#include "rrlib/serialization/tOutputStream.h"
#include "rrlib/serialization/tInputStream.h"
#include "core/port/net/tNetPort.h"
#include "core/port/tAbstractPort.h"
#include "rrlib/finroc_core_utils/stream/tLargeIntermediateStreamBuffer.h"
#include "core/datatype/tNumber.h"
#include "rrlib/serialization/tDataTypeBase.h"
#include "rrlib/finroc_core_utils/tTime.h"
#include "plugins/tcp/tTCPCommand.h"
#include "plugins/tcp/tTCPSettings.h"
#include "core/parameter/tParameterNumeric.h"

namespace finroc
{
namespace tcp
{
const char* tRemoteServer::cCONNECTING = "connecting";
const char* tRemoteServer::cDISCONNECTING = "disconnecting";

tRemoteServer::tRemoteServer(util::tIPSocketAddress isa, const util::tString& name, core::tFrameworkElement* parent, const core::tFrameworkElementTreeFilter& filter_, tTCPPeer* peer_) :
    core::tFrameworkElement(parent, name, core::tCoreFlags::cNETWORK_ELEMENT | core::tCoreFlags::cALLOWS_CHILDREN | (filter_.IsPortOnlyFilter() ? 0 : core::tCoreFlags::cALTERNATE_LINK_ROOT), core::tLockOrderLevels::cREMOTE),
    address(isa),
    bulk(),
    express(),
    connector_thread(util::sThreadUtil::GetThreadSharedPtr(new tConnectorThread(this))),
    tmp_info(),
    filter(filter_),
    remote_port_register(),
    remote_element_register(),
    port_iterator(remote_port_register.GetIterator()),
    elem_iterator(remote_element_register.GetIterator()),
    tmp_match_buffer(),
    server_creation_time(-1),
    peer(peer_),
    global_links(filter_.IsPortOnlyFilter() ? new ::finroc::core::tFrameworkElement(this, "global", core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT | core::tCoreFlags::cGLOBALLY_UNIQUE_LINK | core::tCoreFlags::cALTERNATE_LINK_ROOT, -1) : NULL),
    disconnect_calls(0),
    deleted_soon(false)
{
  core::tRuntimeEnvironment::GetInstance()->AddListener(this);
  connector_thread->Start();
}

void tRemoteServer::Connect()
{
  util::tLock lock1(this);
  status_string = cCONNECTING;

  // reset disconnect count
  disconnect_calls.Set(0);

  // try connecting...
  std::shared_ptr<util::tNetSocket> socket_express = util::tNetSocket::CreateInstance(address);
  std::shared_ptr<util::tNetSocket> socket_bulk = util::tNetSocket::CreateInstance(address);

  // connect
  finroc::util::tGarbageCollector::tFunctor deleter;
  std::shared_ptr<tConnection> express(new tConnection(this, tTCP::cTCP_P2P_ID_EXPRESS), deleter);
  std::shared_ptr<tConnection> bulk(new tConnection(this, tTCP::cTCP_P2P_ID_BULK), deleter);

  // Set bulk and express here, because of other threads that might try to access them
  this->bulk = bulk;
  this->express = express;
  connector_thread->ct_bulk = bulk;
  connector_thread->ct_express = express;

  // init connections...
  try
  {
    express->Connect(socket_express, express);  // express first, since it's required for retrieving ports, which is done in bulk connection
    bulk->Connect(socket_bulk, bulk);
    status_string = NULL;
  }
  catch (const util::tException& e)
  {
    this->bulk.reset();
    this->express.reset();
    throw e;
  }
}

core::tPortCreationInfo tRemoteServer::CreatePCI(const core::tFrameworkElementInfo& port_info)
{
  core::tPortCreationInfo pci(port_info.GetFlags());
  pci.flags = port_info.GetFlags();

  // set queue size
  pci.max_queue_size = port_info.GetStrategy();

  pci.data_type = port_info.GetDataType();
  pci.lock_order = core::tLockOrderLevels::cREMOTE_PORT;

  return pci;
}

void tRemoteServer::Disconnect()
{
  // make sure that disconnect is only called once... prevents deadlocks cleaning up all the threads
  int calls = disconnect_calls.IncrementAndGet();
  if (calls > 1)
  {
    return;
  }

  {
    util::tLock lock2(this);
    status_string = cDISCONNECTING;
    if (bulk.get() != NULL)
    {
      bulk->Disconnect();
      bulk.reset();  // needed afterwards so commmented out
    }
    if (express.get() != NULL)
    {
      express->Disconnect();
      express.reset();  // needed afterwards so commmented out
    }

    // reset subscriptions - possibly delete elements
    port_iterator.Reset();
    for (tProxyPort* pp = port_iterator.Next(); pp != NULL; pp = port_iterator.Next())
    {
      if (peer->DeletePortsOnDisconnect())
      {
        pp->ManagedDelete();
      }
      else
      {
        pp->Reset();
      }
    }
    port_iterator.Reset();

    if (peer->DeletePortsOnDisconnect())
    {
      elem_iterator.Reset();
      for (tProxyFrameworkElement* pp = elem_iterator.Next(); pp != NULL; pp = elem_iterator.Next())
      {
        pp->ManagedDelete();
      }
      elem_iterator.Reset();
    }
    status_string = NULL;
  }
}

util::tString tRemoteServer::FormatRate(int data_rate)
{
  if (data_rate < 1000)
  {
    return util::tStringBuilder("") + data_rate;
  }
  else if (data_rate < 10000000)
  {
    return (data_rate / 1000) + "k";
  }
  else
  {
    return (data_rate / 1000000) + "M";
  }
}

float tRemoteServer::GetConnectionQuality()
{
  if (bulk.get() == NULL || express.get() == NULL || status_string != NULL)
  {
    return 0;
  }
  float ping_time = 0;
  for (int i = 0; i < 2; i++)
  {
    tConnection* c = (i == 0) ? bulk.get() : express.get();
    if (c != NULL)
    {
      if (c->PingTimeExceeed())
      {
        return 0;
      }
      ping_time = std::max(ping_time, static_cast<float>(c->GetAvgPingTime()));
    }
  }
  if (ping_time < 300)
  {
    return 1;
  }
  else if (ping_time > 1300)
  {
    return 0;
  }
  else
  {
    return (static_cast<float>(ping_time) - 300.0f) / 1000.0f;
  }
}

::finroc::core::tFrameworkElement* tRemoteServer::GetFrameworkElement(int handle, int extra_flags, bool port_parent, int parent_handle)
{
  if (handle == core::tRuntimeEnvironment::GetInstance()->GetHandle())    // if parent is runtime environment - should be added as child of remote server
  {
    return this;
  }
  tProxyFrameworkElement* pxe = remote_element_register.Get(-handle);
  if (pxe != NULL)
  {
    assert((pxe->remote_handle == handle));
  }
  else
  {
    if (port_parent)
    {
      pxe = new tProxyFrameworkElement(this, handle, extra_flags, core::tLockOrderLevels::cREMOTE_PORT - 10);
    }
    else
    {
      ::finroc::core::tFrameworkElement* parent = (parent_handle == core::tRuntimeEnvironment::GetInstance()->GetHandle()) ? static_cast< ::finroc::core::tFrameworkElement*>(this) : static_cast< ::finroc::core::tFrameworkElement*>(remote_element_register.Get(-parent_handle));
      assert(((parent != NULL)) && "Framework elements published in the wrong order - server's fault (programming error)");
      pxe = new tProxyFrameworkElement(this, handle, extra_flags, parent->GetLockOrder() + 1);
    }
  }
  return pxe;
}

util::tString tRemoteServer::GetPingString()
{
  if (status_string != NULL)
  {
    return status_string;
  }

  int ping_avg = 0;
  int ping_max = 0;
  int data_rate = 0;
  util::tString s = "ping (avg/max/Rx): ";
  if (bulk.get() == NULL && express.get() == NULL)
  {
    return s + "- ";
  }
  for (int i = 0; i < 2; i++)
  {
    tConnection* c = (i == 0) ? bulk.get() : express.get();
    if (c != NULL)
    {
      if (c->PingTimeExceeed())
      {
        return s + "- ";
      }
      ping_avg = std::max(ping_avg, c->GetAvgPingTime());
      ping_max = std::max(ping_max, c->GetMaxPingTime());
      data_rate += c->GetRx();
    }
  }
  return s + ping_avg + "ms/" + ping_max + "ms/" + FormatRate(data_rate);
}

void tRemoteServer::PrepareDelete()
{
  util::tLock lock1(this);
  core::tRuntimeEnvironment::GetInstance()->RemoveListener(this);
  FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_VERBOSE_1, log_domain, "RemoteServer: Stopping ConnectorThread");
  connector_thread->StopThread();
  try
  {
    connector_thread->Join();
  }
  catch (const util::tInterruptedException& e)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: RemoteServer::prepareDelete() - Interrupted waiting for connector thread.");
  }

  FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "RemoteServer: Disconnecting");
  Disconnect();

  // delete all elements created by this remote server (should be done automatically, actually)
  /*portIterator.reset();
  for (ProxyPort pp = portIterator.next(); pp != null; pp = portIterator.next()) {
      pp.managedDelete();
  }
  elemIterator.reset();
  for (ProxyFrameworkElement pp = elemIterator.next(); pp != null; pp = elemIterator.next()) {
      pp.managedDelete();
  }*/

  ::finroc::core::tFrameworkElement::PrepareDelete();
}

void tRemoteServer::ProcessPortUpdate(core::tFrameworkElementInfo& info)
{
  FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_VERBOSE_2, log_domain, "Received updated FrameworkElementInfo: ", info.ToString());

  // these variables will store element to update
  tProxyFrameworkElement* fe = NULL;
  tProxyPort* port = NULL;

  // find current element
  if (info.IsPort())
  {
    port = remote_port_register.Get(info.GetHandle());
  }
  else
  {
    fe = remote_element_register.Get(-info.GetHandle());
  }

  if (info.op_code == ::finroc::core::tRuntimeListener::cADD)
  {
    // create new framework element
    if (info.IsPort())    // delete old port?
    {
      if (info.GetDataType() == NULL)    // Unknown type... skip
      {
        return;
      }

      if (port != NULL && port->GetRemoteHandle() != info.GetHandle())
      {
        port->ManagedDelete();
        port = NULL;
      }
      if (port == NULL)    // normal case
      {
        port = new tProxyPort(this, info);
      }
      else    // refound port
      {
        printf("refound network port %p %s\n", port, port->GetPort()->GetCDescription());
        {
          util::tLock lock5(port->GetPort());
          port->refound = true;
          port->connection = (info.GetFlags() & core::tPortFlags::cIS_EXPRESS_PORT) > 0 ? express.get() : bulk.get();
          assert(((port->Matches(info))) && "Structure in server changed - that shouldn't happen");
          info.op_code = ::finroc::core::tRuntimeListener::cCHANGE;
          port->UpdateFromPortInfo(info);
        }
      }
    }
    else
    {
      if (fe != NULL && fe->remote_handle != info.GetHandle())    // delete old frameworkElement
      {
        fe->ManagedDelete();
        fe = NULL;
      }
      if (fe == NULL || fe->yet_unknown)    // normal
      {
        fe = static_cast<tProxyFrameworkElement*>(GetFrameworkElement(info.GetHandle(), info.GetFlags(), false, info.GetLink(0)->parent));
        fe->UpdateFromPortInfo(info);
        //fe.yetUnknown = false;
      }
      else if (fe != NULL)    // refound
      {
        {
          util::tLock lock5(fe);
          printf("refound network framework element %p %s\n", fe, fe->GetCDescription());
          fe->refound = true;
          assert(((fe->Matches(info))) && "Structure in server changed - that shouldn't happen");
          info.op_code = ::finroc::core::tRuntimeListener::cCHANGE;
          fe->UpdateFromPortInfo(info);
        }
      }
    }

  }
  else if (info.op_code == ::finroc::core::tRuntimeListener::cCHANGE || info.op_code == core::tFrameworkElementInfo::cEDGE_CHANGE)
  {
    // we're dealing with an existing framework element
    if (info.IsPort())
    {
      if (port == NULL && info.GetDataType() == NULL)    // ignore ports that we did not create, because of unknown type
      {
        return;
      }
      assert((port != NULL));
      port->UpdateFromPortInfo(info);
    }
    else
    {
      assert((fe != NULL));
      fe->UpdateFromPortInfo(info);
    }

  }
  else if (info.op_code == ::finroc::core::tRuntimeListener::cREMOVE)
  {
    // we're dealing with an existing framework element
    if (info.IsPort())
    {
      if (port == NULL && info.GetDataType() == NULL)    // ignore ports that we did not create, because of unknown type
      {
        return;
      }
      assert((port != NULL));
      port->ManagedDelete();
    }
    else
    {
      assert((fe != NULL));
      fe->ManagedDelete();
    }

  }
}

void tRemoteServer::RetrieveRemotePorts(rrlib::serialization::tInputStream* cis, rrlib::serialization::tOutputStream* cos, core::tRemoteTypes* type_lookup, bool new_server)
{
  // recreate/reset monitoring lists if there has already been a connection
  port_iterator.Reset();
  for (tProxyPort* pp = port_iterator.Next(); pp != NULL; pp = port_iterator.Next())
  {
    if (new_server)
    {
      pp->ManagedDelete();  // Delete all ports if we are talking to a new server
    }
    else
    {
      pp->Reset();
    }
  }
  elem_iterator.Reset();
  for (tProxyFrameworkElement* pp = elem_iterator.Next(); pp != NULL; pp = elem_iterator.Next())
  {
    if (new_server)
    {
      pp->ManagedDelete();
    }
    else
    {
      pp->refound = false;  // reset refound flag
    }
  }

  // send opcode & filter information
  //cos.writeByte(TCP.REQUEST_PORT_UPDATE);
  filter.Serialize(*cos);
  //if (TCPSettings.DEBUG_TCP) {
  //  cos.writeInt(TCPSettings.DEBUG_TCP_NUMBER);
  //}
  cos->Flush();

  // retrieve initial port information
  while (cis->ReadByte() != 0)
  {
    tmp_info.Deserialize(*cis, *type_lookup);
    ProcessPortUpdate(tmp_info);
  }
  Init();

  port_iterator.Reset();
  for (tProxyPort* pp = port_iterator.Next(); pp != NULL; pp = port_iterator.Next())
  {
    if (!pp->refound)
    {
      pp->ManagedDelete();  // portUpdated & register remove is performed here
    }
  }
  elem_iterator.Reset();
  for (tProxyFrameworkElement* pp = elem_iterator.Next(); pp != NULL; pp = elem_iterator.Next())
  {
    if (!pp->refound)
    {
      pp->ManagedDelete();  // framework element register remove is performed here
    }
  }

}

void tRemoteServer::RuntimeChange(int8 change_type, core::tFrameworkElement* element)
{
  if (element->IsPort() && change_type != ::finroc::core::tRuntimeListener::cPRE_INIT)
  {
    core::tNetPort* np = (static_cast<core::tAbstractPort*>(element))->FindNetPort(express.get());
    if (np == NULL)
    {
      np = (static_cast<core::tAbstractPort*>(element))->FindNetPort(bulk.get());
    }
    if (np != NULL)
    {
      (static_cast<tProxyPort*>(np))->CheckSubscription();
    }
  }
}

void tRemoteServer::TemporaryDisconnect()
{
  util::tLock lock1(this);

  connector_thread->PauseThread();
  Disconnect();
}

tRemoteServer::tProxyFrameworkElement::tProxyFrameworkElement(tRemoteServer* const outer_class_ptr_, int handle, int extra_flags, int lock_order) :
    core::tFrameworkElement(NULL, "(yet unknown)", core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT | core::tFrameworkElementInfo::FilterParentFlags(extra_flags), lock_order),
    outer_class_ptr(outer_class_ptr_),
    refound(true),
    remote_handle(handle),
    yet_unknown(true)
{
  outer_class_ptr->remote_element_register.Put(-remote_handle, this);
}

bool tRemoteServer::tProxyFrameworkElement::Matches(const core::tFrameworkElementInfo& info)
{
  util::tLock lock1(this);
  if (remote_handle != info.GetHandle() || info.GetLinkCount() != GetLinkCount())
  {
    return false;
  }
  if ((GetAllFlags() & core::tCoreFlags::cCONSTANT_FLAGS) != (info.GetFlags() & core::tCoreFlags::cCONSTANT_FLAGS))
  {
    return false;
  }
  if (GetDescription().Equals(info.GetLink(0)->name))
  {
    return false;
  }
  return true;
}

void tRemoteServer::tProxyFrameworkElement::UpdateFromPortInfo(const core::tFrameworkElementInfo& info)
{
  util::tLock lock1(this);
  if (!IsReady())
  {
    assert(((info.op_code == core::tRuntimeListener::cADD)) && "only add operation may change framework element before initialization");
    assert(((info.GetLinkCount() == 1)) && "Framework elements currently may not be linked");
    SetDescription(info.GetLink(0)->name);
    outer_class_ptr->GetFrameworkElement(info.GetLink(0)->parent, info.GetLink(0)->extra_flags, false, info.GetLink(0)->parent)->AddChild(this);
  }
  if ((info.GetFlags() & core::tCoreFlags::cFINSTRUCTED) > 0)
  {
    SetFlag(core::tCoreFlags::cFINSTRUCTED);
  }
  if ((info.GetFlags() & core::tCoreFlags::cFINSTRUCTABLE_GROUP) > 0)
  {
    SetFlag(core::tCoreFlags::cFINSTRUCTABLE_GROUP);
  }
  yet_unknown = false;
}

tRemoteServer::tProxyPort::tProxyPort(tRemoteServer* const outer_class_ptr_, const core::tFrameworkElementInfo& port_info) :
    tTCPPort(tRemoteServer::CreatePCI(port_info), (port_info.GetFlags() & core::tPortFlags::cIS_EXPRESS_PORT) > 0 ? outer_class_ptr_->express.get() : outer_class_ptr_->bulk.get()),
    outer_class_ptr(outer_class_ptr_),
    refound(true),
    subscription_strategy(-1),
    subscription_rev_push(false),
    subscription_update_time(-1)
{
  this->remote_handle = port_info.GetHandle();
  outer_class_ptr->remote_port_register.Put(this->remote_handle, this);
  UpdateFromPortInfo(port_info);
}

void tRemoteServer::tProxyPort::CheckSubscription()
{
  {
    util::tLock lock2(GetPort()->GetRegistryLock());
    core::tAbstractPort* p = GetPort();
    bool rev_push = p->IsInputPort() && p->IsConnectedToReversePushSources();
    int16 time = GetUpdateIntervalForNet();
    int16 strategy = p->IsInputPort() ? 0 : p->GetStrategy();
    if (!p->IsConnected())
    {
      strategy = -1;
    }

    tRemoteServer::tConnection* c = static_cast<tRemoteServer::tConnection*>(this->connection);

    if (c == NULL)
    {
      subscription_strategy = -1;
      subscription_rev_push = false;
      subscription_update_time = -1;
    }
    else if (strategy == -1 && subscription_strategy > -1)    // disconnect
    {
      c->Unsubscribe(this->remote_handle);
      subscription_strategy = -1;
      subscription_rev_push = false;
      subscription_update_time = -1;
    }
    else if (strategy == -1)
    {
      // still disconnected
    }
    else if (strategy != subscription_strategy || time != subscription_update_time || rev_push != subscription_rev_push)
    {
      c->Subscribe(this->remote_handle, strategy, rev_push, time, p->GetHandle());
      subscription_strategy = strategy;
      subscription_rev_push = rev_push;
      subscription_update_time = time;
    }
  }
}

bool tRemoteServer::tProxyPort::Matches(const core::tFrameworkElementInfo& info)
{
  {
    util::tLock lock2(GetPort());
    if (this->remote_handle != info.GetHandle() || info.GetLinkCount() != GetPort()->GetLinkCount())
    {
      return false;
    }
    if ((GetPort()->GetAllFlags() & core::tCoreFlags::cCONSTANT_FLAGS) != (info.GetFlags() & core::tCoreFlags::cCONSTANT_FLAGS))
    {
      return false;
    }
    for (size_t i = 0u; i < info.GetLinkCount(); i++)
    {
      if (outer_class_ptr->filter.IsPortOnlyFilter())
      {
        GetPort()->GetQualifiedLink(outer_class_ptr->tmp_match_buffer, i);
      }
      else
      {
        outer_class_ptr->tmp_match_buffer.Delete(0, outer_class_ptr->tmp_match_buffer.Length());
        outer_class_ptr->tmp_match_buffer.Append(GetPort()->GetLink(i)->GetDescription());
      }
      if (!outer_class_ptr->tmp_match_buffer.Equals(info.GetLink(i)->name))
      {
        return false;
      }
      // parents are negligible if everything else, matches
    }
    return true;
  }
}

void tRemoteServer::tProxyPort::PrepareDelete()
{
  outer_class_ptr->remote_port_register.Remove(this->remote_handle);
  GetPort()->DisconnectAll();
  CheckSubscription();
  ::finroc::tcp::tTCPPort::PrepareDelete();
}

void tRemoteServer::tProxyPort::Reset()
{
  this->connection = NULL;  // set connection to null
  this->monitored = false;  // reset monitored flag
  refound = false;  // reset refound flag
  PropagateStrategyFromTheNet(static_cast<int16>(0));
  subscription_rev_push = false;
  subscription_update_time = -1;
  subscription_strategy = -1;
}

void tRemoteServer::tProxyPort::UpdateFromPortInfo(const core::tFrameworkElementInfo& port_info)
{
  {
    util::tLock lock2(GetPort()->GetRegistryLock());
    UpdateFlags(port_info.GetFlags());
    GetPort()->SetMinNetUpdateInterval(port_info.GetMinNetUpdateInterval());
    this->update_interval_partner = port_info.GetMinNetUpdateInterval();  // TODO redundant?
    PropagateStrategyFromTheNet(port_info.GetStrategy());

    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_VERBOSE_2, tRemoteServer::log_domain, "Updating port info: ", port_info.ToString());
    if (port_info.op_code == core::tRuntimeListener::cADD)
    {
      assert((!GetPort()->IsReady()));
      if (outer_class_ptr->filter.IsPortOnlyFilter())
      {
        for (int i = 1, n = port_info.GetLinkCount(); i < n; i++)
        {
          core::tFrameworkElement* parent = (port_info.GetLink(i)->extra_flags & core::tCoreFlags::cGLOBALLY_UNIQUE_LINK) > 0 ? outer_class_ptr->global_links : static_cast<core::tFrameworkElement*>(outer_class_ptr);
          GetPort()->Link(parent, port_info.GetLink(i)->name);
        }
        core::tFrameworkElement* parent = (port_info.GetLink(0)->extra_flags & core::tCoreFlags::cGLOBALLY_UNIQUE_LINK) > 0 ? outer_class_ptr->global_links : static_cast<core::tFrameworkElement*>(outer_class_ptr);
        GetPort()->SetDescription(port_info.GetLink(0)->name);
        parent->AddChild(GetPort());
      }
      else
      {
        for (size_t i = 1u; i < port_info.GetLinkCount(); i++)
        {
          core::tFrameworkElement* parent = outer_class_ptr->GetFrameworkElement(port_info.GetLink(i)->parent, port_info.GetLink(i)->extra_flags, true, 0);
          GetPort()->Link(parent, port_info.GetLink(i)->name);
        }
        GetPort()->SetDescription(port_info.GetLink(0)->name);
        outer_class_ptr->GetFrameworkElement(port_info.GetLink(0)->parent, port_info.GetLink(0)->extra_flags, true, 0)->AddChild(GetPort());
      }
    }
    CheckSubscription();
  }
}

tRemoteServer::tConnection::tConnection(tRemoteServer* const outer_class_ptr_, int8 type) :
    tTCPConnection(type, type == tTCP::cTCP_P2P_ID_BULK ? outer_class_ptr_->peer : NULL, type == tTCP::cTCP_P2P_ID_BULK),
    outer_class_ptr(outer_class_ptr_)
{
}

void tRemoteServer::tConnection::Connect(std::shared_ptr<util::tNetSocket>& socket_, std::shared_ptr<tRemoteServer::tConnection>& connection)
{
  this->socket = socket_;

  // write stream id
  std::shared_ptr<util::tLargeIntermediateStreamBuffer> lm_buf(new util::tLargeIntermediateStreamBuffer(socket_->GetSink()));
  this->cos = std::shared_ptr<rrlib::serialization::tOutputStream>(new rrlib::serialization::tOutputStream(lm_buf, this->update_times));
  this->cos->WriteByte(this->type);
  //RemoteTypes.serializeLocalDataTypes(cos);
  this->cos->WriteType(core::tNumber::cTYPE);  // initialize type register
  bool bulk = this->type == tTCP::cTCP_P2P_ID_BULK;
  util::tString type_string = GetConnectionTypeString();
  this->cos->WriteBoolean(bulk);
  this->cos->Flush();

  // initialize core streams
  this->cis = std::shared_ptr<rrlib::serialization::tInputStream>(new rrlib::serialization::tInputStream(socket_->GetSource(), this->update_times));
  this->cis->SetTimeout(1000);
  this->time_base = this->cis->ReadLong();  // Timestamp that remote runtime was created - and relative to which time is encoded in this stream
  //updateTimes.deserialize(cis);
  rrlib::serialization::tDataTypeBase dt = this->cis->ReadType();
  assert((dt == core::tNumber::cTYPE));
  this->cis->SetTimeout(-1);

  std::shared_ptr<tTCPConnection::tReader> listener = util::sThreadUtil::GetThreadSharedPtr(new tTCPConnection::tReader(this, util::tStringBuilder("TCP Client ") + type_string + "-Listener for " + outer_class_ptr->GetDescription()));
  this->reader = listener;
  std::shared_ptr<tTCPConnection::tWriter> writer = util::sThreadUtil::GetThreadSharedPtr(new tTCPConnection::tWriter(this, util::tStringBuilder("TCP Client ") + type_string + "-Writer for " + outer_class_ptr->GetDescription()));
  this->writer = writer;

  if (bulk)
  {
    bool new_server = (outer_class_ptr->server_creation_time < 0) || (outer_class_ptr->server_creation_time != this->time_base);
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, (new_server ? "Connecting" : "Reconnecting"), " to server ", socket_->GetRemoteSocketAddress().ToString(), "...");
    outer_class_ptr->RetrieveRemotePorts(this->cis.get(), this->cos.get(), this->update_times.get(), new_server);
  }

  // start incoming data listener thread
  listener->LockObject(connection);
  listener->Start();

  // start writer thread
  writer->LockObject(connection);
  writer->Start();
}

tTCPPort* tRemoteServer::tConnection::LookupPortForCallHandling(int port_index)
{
  core::tAbstractPort* ap = core::tRuntimeEnvironment::GetInstance()->GetPort(port_index);
  tTCPPort* p = NULL;
  if (ap != NULL)
  {
    p = static_cast<tTCPPort*>(ap->AsNetPort());
    assert((p != NULL));
  }
  return p;
}

void tRemoteServer::tConnection::ProcessRequest(int8 op_code)
{
  int port_index = 0;
  core::tNetPort* p = NULL;
  //MethodCall mc = null;
  core::tAbstractPort* ap = NULL;

  switch (op_code)
  {
  case tTCP::cCHANGE_EVENT:

    // read port index and retrieve proxy port
    port_index = this->cis->ReadInt();
    this->cis->ReadSkipOffset();
    ap = core::tRuntimeEnvironment::GetInstance()->GetPort(port_index);
    if (ap != NULL)
    {
      p = ap->AsNetPort();
      assert((p != NULL));
    }

    // read time stamp... will be optimized in the future to save space
    //long timestamp = readTimestamp();

    // write to proxy port
    if (ap != NULL)
    {
      // make sure, "our" port is not deleted while we use it
      {
        util::tLock lock4(ap);
        if (!ap->IsReady())
        {
          this->cis->ToSkipTarget();
        }
        else
        {
          int8 changed_flag = this->cis->ReadByte();
          this->cis->SetFactory(p->GetPort());
          p->ReceiveDataFromStream(*this->cis, util::tTime::GetCoarse(), changed_flag);
          this->cis->SetFactory(NULL);
        }
      }
    }
    else
    {
      this->cis->ToSkipTarget();
    }
    break;

  case tTCP::cPORT_UPDATE:

    outer_class_ptr->tmp_info.Deserialize(*this->cis, *this->update_times);
    outer_class_ptr->ProcessPortUpdate(outer_class_ptr->tmp_info);
    outer_class_ptr->Init();
    break;

  default:
    throw util::tException(util::tStringBuilder("Client Listener does not accept this opcode: ") + op_code);
  }
}

bool tRemoteServer::tConnection::SendData(int64 start_time)
{
  // send port data
  return ::finroc::tcp::tTCPConnection::SendDataPrototype(start_time, tTCP::cSET);

}

void tRemoteServer::tConnection::Subscribe(int index, int16 strategy, bool reverse_push, int16 update_interval, int local_index)
{
  tTCPCommand* command = tTCP::GetUnusedTCPCommand();
  command->op_code = tTCP::cSUBSCRIBE;
  command->remote_handle = index;
  command->strategy = strategy;
  command->reverse_push = reverse_push;
  command->update_interval = update_interval;
  command->local_index = local_index;
  SendCall(command);
  //command.genericRecycle();
}

void tRemoteServer::tConnection::Unsubscribe(int index)
{
  tTCPCommand* command = tTCP::GetUnusedTCPCommand();
  command->op_code = tTCP::cUNSUBSCRIBE;
  command->remote_handle = index;
  SendCall(command);
  //command.genericRecycle();
}

tRemoteServer::tConnectorThread::tConnectorThread(tRemoteServer* const outer_class_ptr_) :
    core::tCoreLoopThreadBase(tTCPSettings::cCONNECTOR_THREAD_LOOP_INTERVAL, false, false),
    outer_class_ptr(outer_class_ptr_),
    last_subscription_update(0),
    ct_bulk(),
    ct_express()
{
  SetName(util::tStringBuilder("TCP Connector Thread for ") + outer_class_ptr->GetDescription());
  FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_VERBOSE_1, log_domain, "Creating ", GetName());
  //this.setPriority(1); // low priority
}

void tRemoteServer::tConnectorThread::MainLoopCallback()
{
  if (ct_bulk.get() != NULL && ct_bulk->Disconnecting())
  {
    ct_bulk.reset();
  }
  if (ct_express.get() != NULL && ct_express->Disconnecting())
  {
    ct_express.reset();
  }

  if (ct_bulk.get() == NULL && ct_express.get() == NULL && outer_class_ptr->bulk.get() == NULL && outer_class_ptr->express.get() == NULL)
  {
    // Try connecting
    try
    {
      outer_class_ptr->Connect();
    }
    catch (const util::tConnectException& e)
    {
      ::finroc::util::tThread::Sleep(2000);
      if (outer_class_ptr->bulk.get() == NULL)
      {
        ct_bulk.reset();
      }
      if (outer_class_ptr->express.get() == NULL)
      {
        ct_express.reset();
      }
    }
    catch (const util::tException& e)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_WARNING, log_domain, e);
      if (outer_class_ptr->bulk.get() == NULL)
      {
        ct_bulk.reset();
      }
      if (outer_class_ptr->express.get() == NULL)
      {
        ct_express.reset();
      }
    }

  }
  else if (ct_bulk.get() != NULL && ct_express.get() != NULL)
  {
    try
    {
      // check ping times
      int64 start_time = util::tSystem::CurrentTimeMillis();
      int64 may_wait = tTCPSettings::GetInstance()->critical_ping_threshold.GetValue();
      may_wait = std::min(may_wait, ct_express->CheckPingForDisconnect());
      may_wait = std::min(may_wait, ct_bulk->CheckPingForDisconnect());

      if (start_time > last_subscription_update + tTCPSettings::cCONNECTOR_THREAD_SUBSCRIPTION_UPDATE_INTERVAL)
      {
        last_subscription_update = start_time;

      }

      // wait remaining uncritical time
      int64 wait_for = may_wait - (util::tSystem::CurrentTimeMillis() - start_time);
      if (wait_for > 0)
      {
        ::finroc::util::tThread::Sleep(wait_for);
      }

    }
    catch (const util::tException& e)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG_WARNING, log_domain, e);
    }
  }
}

} // namespace finroc
} // namespace tcp

