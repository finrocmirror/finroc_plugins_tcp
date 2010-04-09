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
#include "finroc_core_utils/tGarbageCollector.h"

#include "tcp/tRemoteServer.h"
#include "core/tCoreFlags.h"
#include "core/tRuntimeEnvironment.h"
#include "finroc_core_utils/thread/sThreadUtil.h"
#include "tcp/tTCP.h"
#include "core/port/tPortFlags.h"
#include "core/port/net/tNetPort.h"
#include "core/port/tAbstractPort.h"
#include "tcp/tTCPSettings.h"
#include "finroc_core_utils/stream/tLargeIntermediateStreamBuffer.h"
#include "core/portdatabase/tDataTypeRegister.h"
#include "finroc_core_utils/tTime.h"
#include "tcp/tTCPCommand.h"

namespace finroc
{
namespace tcp
{
tRemoteServer::tRemoteServer(util::tIPSocketAddress isa, const util::tString& name, core::tFrameworkElement* parent, const core::tFrameworkElementTreeFilter& filter_, tTCPPeer* peer_) :
    core::tFrameworkElement(name, parent, core::tCoreFlags::cNETWORK_ELEMENT | core::tCoreFlags::cALLOWS_CHILDREN | (filter_.IsPortOnlyFilter() ? 0 : core::tCoreFlags::cALTERNATE_LINK_ROOT)),
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
    global_links(filter_.IsPortOnlyFilter() ? new ::finroc::core::tFrameworkElement("global", this, core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT | core::tCoreFlags::cGLOBALLY_UNIQUE_LINK | core::tCoreFlags::cALTERNATE_LINK_ROOT) : NULL)
{
  core::tRuntimeEnvironment::GetInstance()->AddListener(this);
  connector_thread->Start();
}

void tRemoteServer::Connect()
{
  util::tLock lock1(obj_synch);

  // try connecting...
  ::std::tr1::shared_ptr<util::tNetSocket> socket_express = util::tNetSocket::CreateInstance(address);
  ::std::tr1::shared_ptr<util::tNetSocket> socket_bulk = util::tNetSocket::CreateInstance(address);

  // connect
  finroc::util::tGarbageCollector::tFunctor deleter;
  std::tr1::shared_ptr<tConnection> express(new tConnection(this, tTCP::cTCP_P2P_ID_EXPRESS), deleter);
  std::tr1::shared_ptr<tConnection> bulk(new tConnection(this, tTCP::cTCP_P2P_ID_BULK), deleter);

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

  // unset shared flag
  //pci.setFlag(PortFlags.NETWORK_PORT, true);

  // unset copy data flags (data is always copied)
  //pci.flags = PortFlags.setFlag(pci.flags, PortFlags.COPY_DATA, false);
  //pci.flags = PortFlags.setFlag(pci.flags, PortFlags.COPY_REVERSE_DATA, false);

  // always create send buffers (for deserialization)
  //pci.flags = PortFlags.setFlag(pci.flags, PortFlags.OWNS_SEND_BUFFERS, true);

  // set queue size
  pci.max_queue_size = port_info.GetStrategy();

  pci.data_type = port_info.GetDataType();

  return pci;
}

void tRemoteServer::Disconnect()
{
  util::tLock lock1(obj_synch);
  if (bulk != NULL)
  {
    bulk->Disconnect();
    bulk.reset();  // needed afterwards so commmented out
  }
  if (express != NULL)
  {
    express->Disconnect();
    express.reset();  // needed afterwards so commmented out
  }

  // reset subscriptions
  port_iterator.Reset();
  for (tProxyPort* pp = port_iterator.Next(); pp != NULL; pp = port_iterator.Next())
  {
    pp->Reset();
  }
  port_iterator.Reset();
  //    for (ProxyPort pp = portIterator.next(); pp != null; pp = portIterator.next()) {
  //      pp.subscriptionQueueLength = 0;
  //    }
}

::finroc::core::tFrameworkElement* tRemoteServer::GetFrameworkElement(int handle, int extra_flags)
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
    pxe = new tProxyFrameworkElement(this, handle, extra_flags);
  }
  return pxe;
}

void tRemoteServer::PrepareDelete()
{
  util::tLock lock1(obj_synch);
  core::tRuntimeEnvironment::GetInstance()->RemoveListener(this);
  connector_thread->StopThread();
  /*try {
    connectorThread.join();
  } catch (InterruptedException e) {
    e.printStackTrace();
  }*/
  Disconnect();

  // delete all elements created by this remote server
  port_iterator.Reset();
  for (tProxyPort* pp = port_iterator.Next(); pp != NULL; pp = port_iterator.Next())
  {
    pp->ManagedDelete();
  }
  elem_iterator.Reset();
  for (tProxyFrameworkElement* pp = elem_iterator.Next(); pp != NULL; pp = elem_iterator.Next())
  {
    pp->ManagedDelete();
  }

  ::finroc::core::tFrameworkElement::PrepareDelete();
}

void tRemoteServer::ProcessPortUpdate(core::tFrameworkElementInfo& info)
{
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

      if (port != NULL && port->remote_handle != info.GetHandle())
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
        port->refound = true;
        port->connection = (info.GetFlags() & core::tPortFlags::cIS_EXPRESS_PORT) > 0 ? express.get() : bulk.get();
        assert(((port->Matches(info))) && "Structure in server changed - that shouldn't happen");
        info.op_code = ::finroc::core::tRuntimeListener::cCHANGE;
        port->UpdateFromPortInfo(info);
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
        fe = static_cast<tProxyFrameworkElement*>(GetFrameworkElement(info.GetHandle(), info.GetFlags()));
        fe->UpdateFromPortInfo(info);
        //fe.yetUnknown = false;
      }
      else if (fe != NULL)    // refound
      {
        printf("refound network framework element %p %s\n", fe, fe->GetCDescription());
        fe->refound = true;
        assert(((fe->Matches(info))) && "Structure in server changed - that shouldn't happen");
        info.op_code = ::finroc::core::tRuntimeListener::cCHANGE;
        fe->UpdateFromPortInfo(info);
      }
    }

  }
  else if (info.op_code == ::finroc::core::tRuntimeListener::cCHANGE)
  {
    // we're dealing with an existing framework element
    assert((fe != NULL || port != NULL));
    if (info.IsPort())
    {
      port->UpdateFromPortInfo(info);
    }
    else
    {
      fe->UpdateFromPortInfo(info);
    }

  }
  else if (info.op_code == ::finroc::core::tRuntimeListener::cREMOVE)
  {
    // we're dealing with an existing framework element
    assert((fe != NULL || port != NULL));
    if (info.IsPort())
    {
      port->ManagedDelete();
    }
    else
    {
      fe->ManagedDelete();
    }

  }
}

void tRemoteServer::RetrieveRemotePorts(core::tCoreInput* cis, core::tCoreOutput* cos, core::tRemoteTypes* type_lookup, bool new_server)
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
    tmp_info.Deserialize(cis, *type_lookup);
    //System.out.println("Received info: " + tmpInfo.toString());
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

tRemoteServer::tProxyFrameworkElement::tProxyFrameworkElement(tRemoteServer* const outer_class_ptr_, int handle, int extra_flags) :
    core::tFrameworkElement("(yet unknown)", NULL, core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT | (extra_flags & core::tFrameworkElementInfo::cPARENT_FLAGS_TO_STORE)),
    outer_class_ptr(outer_class_ptr_),
    refound(true),
    remote_handle(handle),
    yet_unknown(true)
{
  outer_class_ptr->remote_element_register.Put(-remote_handle, this);
}

bool tRemoteServer::tProxyFrameworkElement::Matches(const core::tFrameworkElementInfo& info)
{
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
  if (!IsReady())
  {
    assert(((info.op_code == core::tRuntimeListener::cADD)) && "only add operation may change framework element before initialization");
    assert(((info.GetLinkCount() == 1)) && "Framework elements currently may not be linked");
    //        for (int i = 1; i < info.getLinks().size(); i++) {
    //          ProxyFrameworkElement pxe = getFrameworkElement(info.getParents().get(i));
    //          pxe.link(this, info.getLinks().get(i));
    //        }
    SetDescription(info.GetLink(0)->name);
    outer_class_ptr->GetFrameworkElement(info.GetLink(0)->parent, info.GetLink(0)->extra_flags)->AddChild(this);
  }
  yet_unknown = false;
}

tRemoteServer::tProxyPort::tProxyPort(tRemoteServer* const outer_class_ptr_, const core::tFrameworkElementInfo& port_info) :
    tTCPPort(tRemoteServer::CreatePCI(port_info), (port_info.GetFlags() & core::tPortFlags::cIS_EXPRESS_PORT) > 0 ? outer_class_ptr_->express.get() : outer_class_ptr_->bulk.get()),
    outer_class_ptr(outer_class_ptr_),
    refound(true),
    subscription_strategy(-1),
    subscription_rev_push(false),
    subscription_update_time(-1),
    obj_synch()
{
  this->remote_handle = port_info.GetHandle();
  outer_class_ptr->remote_port_register.Put(this->remote_handle, this);
  UpdateFromPortInfo(port_info);
}

void tRemoteServer::tProxyPort::CheckSubscription()
{
  util::tLock lock1(obj_synch);
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

bool tRemoteServer::tProxyPort::Matches(const core::tFrameworkElementInfo& info)
{
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

void tRemoteServer::tProxyPort::PrepareDelete()
{
  outer_class_ptr->remote_port_register.Remove(this->remote_handle);
  GetPort()->DisconnectAll();
  CheckSubscription();
  ::finroc::tcp::tTCPPort::PrepareDelete();
}

void tRemoteServer::tProxyPort::PropagateStrategyOverTheNet()
{
  //      if (getPort().getStrategy() == -1) {
  //        ((Connection)connection).unsubscribe(remoteHandle);
  //        connected = false;
  //      } else {
  //        ((Connection)connection).subscribe(remoteHandle, getPort().getStrategy(), getPort().isConnectedToReversePushSources(), getUpdateIntervalForNet(), getPort().getHandle());
  //        connected = true;
  //      }
  CheckSubscription();
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
  UpdateFlags(port_info.GetFlags());
  GetPort()->SetMinNetUpdateInterval(port_info.GetMinNetUpdateInterval());
  this->update_interval_partner = port_info.GetMinNetUpdateInterval();  // TODO redundant?
  PropagateStrategyFromTheNet(port_info.GetStrategy());
  if (tTCPSettings::cDISPLAY_INCOMING_PORT_UPDATES->Get())
  {
    util::tSystem::out.Println(util::tStringBuilder("Updating port info: ") + port_info.ToString());
  }
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
        core::tFrameworkElement* parent = outer_class_ptr->GetFrameworkElement(port_info.GetLink(i)->parent, port_info.GetLink(i)->extra_flags);
        GetPort()->Link(parent, port_info.GetLink(i)->name);
      }
      GetPort()->SetDescription(port_info.GetLink(0)->name);
      outer_class_ptr->GetFrameworkElement(port_info.GetLink(0)->parent, port_info.GetLink(0)->extra_flags)->AddChild(GetPort());
    }
  }
  CheckSubscription();
}

tRemoteServer::tConnection::tConnection(tRemoteServer* const outer_class_ptr_, int8 type) :
    tTCPConnection(type, type == tTCP::cTCP_P2P_ID_BULK ? outer_class_ptr_->peer : NULL, type == tTCP::cTCP_P2P_ID_BULK),
    outer_class_ptr(outer_class_ptr_)
{
}

void tRemoteServer::tConnection::Connect(::std::tr1::shared_ptr<util::tNetSocket> socket_, ::std::tr1::shared_ptr<tRemoteServer::tConnection> connection)
{
  this->socket = socket_;

  // write stream id
  ::std::tr1::shared_ptr<util::tLargeIntermediateStreamBuffer> lm_buf(new util::tLargeIntermediateStreamBuffer(socket_->GetSink()));
  this->cos = ::std::tr1::shared_ptr<core::tCoreOutput>(new core::tCoreOutput(lm_buf));
  this->cos->WriteByte(this->type);
  core::tRemoteTypes::SerializeLocalDataTypes(core::tDataTypeRegister::GetInstance(), this->cos.get());
  bool bulk = this->type == tTCP::cTCP_P2P_ID_BULK;
  util::tString type_string = GetConnectionTypeString();
  this->cos->WriteBoolean(bulk);
  this->cos->Flush();

  // initialize core streams
  this->cis = ::std::tr1::shared_ptr<core::tCoreInput>(new core::tCoreInput(socket_->GetSource()));
  this->cis->SetTypeTranslation(&(this->update_times));
  this->time_base = this->cis->ReadLong();  // Timestamp that remote runtime was created - and relative to which time is encoded in this stream
  this->update_times.Deserialize(this->cis.get());

  ::std::tr1::shared_ptr<tTCPConnection::tReader> listener = util::sThreadUtil::GetThreadSharedPtr(new tTCPConnection::tReader(this, util::tStringBuilder("TCP Client ") + type_string + "-Listener for " + outer_class_ptr->GetDescription()));
  ::std::tr1::shared_ptr<tTCPConnection::tWriter> writer = util::sThreadUtil::GetThreadSharedPtr(new tTCPConnection::tWriter(this, util::tStringBuilder("TCP Client ") + type_string + "-Writer for " + outer_class_ptr->GetDescription()));
  this->writer = writer;

  if (bulk)
  {
    bool new_server = (outer_class_ptr->server_creation_time < 0) || (outer_class_ptr->server_creation_time != this->time_base);
    if (!new_server)
    {
      util::tSystem::out.Print("Re-");
    }
    util::tSystem::out.Println(util::tStringBuilder("Connecting to server ") + socket_->GetRemoteSocketAddress().ToString() + "...");
    outer_class_ptr->RetrieveRemotePorts(this->cis.get(), this->cos.get(), &(this->update_times), new_server);
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
      int8 changed_flag = this->cis->ReadByte();
      this->cis->SetBufferSource(p->GetPort());
      p->ReceiveDataFromStream(this->cis.get(), util::tTime::GetCoarse(), changed_flag);
      this->cis->SetBufferSource(NULL);
    }
    else
    {
      this->cis->ToSkipTarget();
    }
    break;

    //      case TCP.METHODCALL:
    //
    //        if (ap == null) {
    //          if (TCPSettings.DISPLAY_INCOMING_TCP_SERVER_COMMANDS.get()) {
    //            System.out.println("Skipping Incoming Server Command: Method call for portIndex " + portIndex);
    //          }
    //          cis.toSkipTarget();
    //        }
    //
    //        // okay... this is handled asynchronously now
    //        // create/decode call
    //        cis.setBufferSource(p.getPort());
    //        // lookup method type
    //        DataType methodType = cis.readType();
    //        if (methodType == null || (!methodType.isMethodType())) {
    //          cis.toSkipTarget();
    //        } else {
    //          mc = ThreadLocalCache.getFast().getUnusedMethodCall();
    //          mc.deserializeCall(cis, methodType);
    //
    //          // process call
    //          if (TCPSettings.DISPLAY_INCOMING_TCP_SERVER_COMMANDS.get()) {
    //            System.out.println("Incoming Server Command: Method call " + (p != null ? p.getPort().getQualifiedName() : handle));
    //          }
    //          p.handleCallReturnFromNet(mc);
    //        }
    //        cis.setBufferSource(null);
    //
    //        break;

    //      case TCP.PULLCALL:
    //
    //        handlePullCall(p, handle, RemoteServer.this);
    //        break;

  case tTCP::cPORT_UPDATE:

    outer_class_ptr->tmp_info.Deserialize(this->cis.get(), this->update_times);
    outer_class_ptr->ProcessPortUpdate(outer_class_ptr->tmp_info);
    break;

  default:
    throw util::tException(util::tStringBuilder("Client Listener does not accept this opcode: ") + op_code);
  }
}

bool tRemoteServer::tConnection::SendData(int64 start_time)
{
  // send port data
  return ::finroc::tcp::tTCPConnection::SendDataPrototype(start_time, tTCP::cSET);

  /*
  boolean requestAcknowledgement = false;

  @Ptr ArrayWrapper<ProxyPort> it = monitoredPorts.getIterable();
  for (@SizeT int i = 0, n = it.size(); i < n; i++) {
    ProxyPort pp = it.get(i);
    if (pp.lastUpdate + pp.getPort().getMinNetUpdateInterval() > startTime) {
      // value cannot be written in this iteration due to minimal update rate
      notifyWriter();

    } else if (pp.getPort().hasChanged()) {
      pp.getPort().resetChanged();
      requestAcknowledgement = true;

      // execute/write set command to stream
      cos.writeByte(TCP.SET);
      cos.writeInt(pp.getRemoteHandle());
      cos.writeSkipOffsetPlaceholder();
      pp.writeDataToNetwork(cos, startTime);
      cos.skipTargetHere();
      terminateCommand();
    }
  }
  // release any locks we acquired
  ThreadLocalCache.get().releaseAllLocks();

  return requestAcknowledgement;
  */
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
  //this.setPriority(1); // low priority
}

void tRemoteServer::tConnectorThread::MainLoopCallback()
{
  if (ct_bulk != NULL && ct_bulk->Disconnecting())
  {
    ct_bulk.reset();
  }
  if (ct_express != NULL && ct_express->Disconnecting())
  {
    ct_express.reset();
  }

  if (ct_bulk == NULL && ct_express == NULL && outer_class_ptr->bulk == NULL && outer_class_ptr->express == NULL)
  {
    // Try connecting
    try
    {
      outer_class_ptr->Connect();
    }
    catch (const util::tConnectException& e)
    {
      ::finroc::util::tThread::Sleep(2000);
    }
    catch (const util::tException& e)
    {
      e.PrintStackTrace();
    }

  }
  else if (ct_bulk != NULL && ct_express != NULL)
  {
    try
    {
      // check ping times
      int64 start_time = util::tSystem::CurrentTimeMillis();
      int64 may_wait = tTCPSettings::critical_ping_threshold->Get();
      may_wait = util::tMath::Min(may_wait, ct_express->CheckPingForDisconnect());
      may_wait = util::tMath::Min(may_wait, ct_bulk->CheckPingForDisconnect());

      if (start_time > last_subscription_update + tTCPSettings::cCONNECTOR_THREAD_SUBSCRIPTION_UPDATE_INTERVAL)
      {
        last_subscription_update = start_time;

        //            // Update subscriptions (should not take significant time)
        //            synchronized(RemoteServer.this) {
        //              if (isDeleted()) {
        //                return;
        //              }
        //
        //              ci.reset();
        //              for (ProxyPort pp = ci.next(); pp != null; pp = ci.next()) {
        //                AbstractPort ap = pp.getPort();
        //                Connection connection = ap.getFlag(PortFlags.IS_EXPRESS_PORT) ? express : bulk;
        //
        //                // Update Subscriptions
        //                short updateTime = pp.getMinNetUpdateIntervalForSubscription();
        //                if (ap.isOutputPort()) { // remote output port
        //                  if (ap.hasActiveEdges()) {
        //                    int qlen = ap.getMaxTargetQueueLength();
        //                    if (qlen != pp.subscriptionQueueLength || updateTime != pp.subscriptionUpdateTime) {
        //                      pp.subscriptionQueueLength = qlen;
        //                      pp.subscriptionUpdateTime = updateTime;
        //                      connection.subscribe(pp.remoteHandle, qlen, updateTime, ap.getHandle());
        //                    }
        //                  } else if (pp.subscriptionQueueLength > 0) {
        //                    pp.subscriptionQueueLength = 0;
        //                    connection.unsubscribe(pp.remoteHandle);
        //                  }
        //                } else { // remote input port and local io port(s)
        //                  if (ap.hasActiveEdgesReverse()) {
        //                    if (pp.subscriptionQueueLength != 1 || updateTime != pp.subscriptionUpdateTime) {
        //                      pp.subscriptionQueueLength = 1;
        //                      pp.subscriptionUpdateTime = updateTime;
        //                      connection.subscribe(pp.remoteHandle, 1, updateTime, ap.getHandle());
        //                    }
        //                  } else if (pp.subscriptionQueueLength > 0) {
        //                    pp.subscriptionQueueLength = 0;
        //                    connection.unsubscribe(pp.remoteHandle);
        //                  }
        //                }
        //              }
        //            }
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
      e.PrintStackTrace();
    }
  }
}

} // namespace finroc
} // namespace tcp

