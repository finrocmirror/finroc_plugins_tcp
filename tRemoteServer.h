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
#include "rrlib/finroc_core_utils/tJCBase.h"

#ifndef PLUGINS__TCP__TREMOTESERVER_H
#define PLUGINS__TCP__TREMOTESERVER_H

#include "rrlib/finroc_core_utils/net/tIPSocketAddress.h"
#include "core/datatype/tFrameworkElementInfo.h"
#include "core/tFrameworkElementTreeFilter.h"
#include "core/port/net/tRemoteCoreRegister.h"
#include "core/port/tPortCreationInfo.h"
#include "core/tFrameworkElement.h"
#include "core/tRuntimeListener.h"
#include "plugins/tcp/tTCPPort.h"
#include "rrlib/finroc_core_utils/net/tNetSocket.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "plugins/tcp/tTCPConnection.h"
#include "core/thread/tCoreLoopThreadBase.h"

namespace finroc
{
namespace core
{
class tCoreInput;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace tcp
{
class tTCPPeer;

/*!
 * \author Max Reichardt
 *
 * Class that stores information about and can be used to access
 * TCP Server running in another runtime environment.
 *
 * Thread safety considerations:
 *  Many Threads are operating on our objects. Before deleting connection or when disconnecting they
 *  should all be stopped to avoid any race conditions.
 *  Furthermore, ports may be deleted via remote Runtime changes. This can happen while other threads
 *  are using these ports. Therefore, critical port operations should be executed in synchronized context
 *  - as well as any deleting.
 */
class tRemoteServer : public core::tFrameworkElement, public core::tRuntimeListener
{
public:
  class tProxyFrameworkElement; // inner class forward declaration
public:
  class tProxyPort; // inner class forward declaration
public:
  class tConnection; // inner class forward declaration

  /*!
   * This thread reconnects if connection was interrupted and updates subscriptions.
   */
  class tConnectorThread : public core::tCoreLoopThreadBase
  {
    friend class tRemoteServer;
  private:

    // Outer class RemoteServer
    tRemoteServer* const outer_class_ptr;

    /*! Timestamp of last subscription update */
    int64 last_subscription_update;

    /*! Bulk and Express Connections to server - copy for connector thread */
    ::std::shared_ptr<tRemoteServer::tConnection> ct_bulk, ct_express;

  public:

    tConnectorThread(tRemoteServer* const outer_class_ptr_);

    virtual void MainLoopCallback();

  };

private:

  /*! Network address */
  util::tIPSocketAddress address;

  /*! Bulk and Express Connections to server */
  ::std::shared_ptr<tConnection> bulk, express;

  /*! This thread reconnects disconnected Remote Nodes and updates subscriptions */
  ::std::shared_ptr<tConnectorThread> connector_thread;

  /*! Temporary buffer with port information */
  core::tFrameworkElementInfo tmp_info;

  /*! Filter that specifies which framework element we're interested in */
  core::tFrameworkElementTreeFilter filter;

  /*! Lookup for remote framework elements (currently not ports) - similar to remote CoreRegister */
  core::tRemoteCoreRegister<tProxyPort*> remote_port_register;

  /*! Lookup for remote framework elements (currently not ports) - similar to remote CoreRegister */
  core::tRemoteCoreRegister<tProxyFrameworkElement*> remote_element_register;

  /*! Iterator for port register (only used by reader thread) */
  core::tRemoteCoreRegister< ::finroc::tcp::tRemoteServer::tProxyPort*>::tIterator port_iterator;

  /*! Iterator for framework element register (only used by bulk reader thread) */
  core::tRemoteCoreRegister< ::finroc::tcp::tRemoteServer::tProxyFrameworkElement*>::tIterator elem_iterator;

  /*! Temporary buffer for match checks (only used by bulk reader or connector thread) */
  util::tStringBuilder tmp_match_buffer;

  /*! Timestamp of when server was created - used to identify whether we are still communicating with same instance after connection loss */
  int64 server_creation_time;

  /*! Peer that this server belongs to */
  tTCPPeer* peer;

  /*! If this is a port-only-client: Framework element that contains all global links */
  ::finroc::core::tFrameworkElement* global_links;

  /*! Number of times disconnect was called, since last connect */
  util::tAtomicInt disconnect_calls;

  /*! Set to true when server will soon be deleted */
  bool deleted_soon;

public:

  /*! Log domain for this class */
  RRLIB_LOG_CREATE_NAMED_DOMAIN(log_domain, "tcp");

private:

  /*!
   * Connect to remote server
   */
  void Connect();

  /*!
   * (Belongs to ProxyPort)
   *
   * Create Port Creation info from PortInfo class.
   * Except from Shared flag port will be identical to original port.
   *
   * \param port_info Port Information
   * \return Port Creation info
   */
  static core::tPortCreationInfo CreatePCI(const core::tFrameworkElementInfo& port_info);

  /*!
   * Disconnect from remote server
   */
  void Disconnect();

  /*!
   * \param data_rate Data Rate
   * \return Formatted Data Rate
   */
  static util::tString FormatRate(int data_rate);

  /*!
   * Process incoming framework element change
   *
   * \param cFramework element change information
   */
  void ProcessPortUpdate(core::tFrameworkElementInfo& info);

  /*!
   * Fetches ports and possibly runtime element from remote runtime environment
   *
   * \param cis CoreInput to use
   * \param cos CoreOutput to write request to
   * \param type_lookup Remote Type Database
   * \param cAre we communicating with a new server?
   */
  void RetrieveRemotePorts(core::tCoreInput* cis, core::tCoreOutput* cos, core::tRemoteTypes* type_lookup, bool new_server);

protected:

  virtual void PrepareDelete();

public:

  /*!
   * \param isa Network address
   * \param name Unique server name
   * \param parent Parent framework element
   * \param filter Filter that specifies which framework element we're interested in
   * \param peer Peer that this server belongs to
   */
  tRemoteServer(util::tIPSocketAddress isa, const util::tString& name, core::tFrameworkElement* parent, const core::tFrameworkElementTreeFilter& filter_, tTCPPeer* peer_);

  /*!
   * \return true when server will soon be deleted
   */
  inline bool DeletedSoon()
  {
    return deleted_soon;
  }

  /*!
   * Early preparations for deleting this
   */
  inline void EarlyDeletingPreparations()
  {
    connector_thread->StopThread();
    deleted_soon = true;
  }

  /*!
   * \return Connection quality (see ExternalConnection)
   */
  float GetConnectionQuality();

  /*!
   * Returns framework element with specified handle.
   * Creates one if it doesn't exist.
   *
   * \param handle Remote Handle of parent
   * \param extra_flags Any extra flags of parent to keep
   * \param port_parent Parent of a port?
   * \param parent_handle Handle of parent (only necessary, when not unknown parent of a port)
   * \return Framework element.
   */
  ::finroc::core::tFrameworkElement* GetFrameworkElement(int handle, int extra_flags, bool port_parent, int parent_handle);

  /*!
   * \return Address of connection partner
   */
  inline util::tIPSocketAddress GetPartnerAddress()
  {
    return address;
  }

  /*!
   * \return String containing ping times
   */
  util::tString GetPingString();

  /*!
   * Reconnect after temporary disconnect
   */
  inline void Reconnect()
  {
    util::tLock lock2(this);
    connector_thread->ContinueThread();
  }

  virtual void RuntimeChange(int8 change_type, core::tFrameworkElement* element);

  virtual void RuntimeEdgeChange(int8 change_type, core::tAbstractPort* source, core::tAbstractPort* target)
  {
    RuntimeChange(change_type, source);
    RuntimeChange(change_type, target);
  }

  /*!
   * Disconnects and pauses connector thread
   */
  void TemporaryDisconnect();

public:

  /*!
   * \author Max Reichardt
   *
   * Dummy framework element for clients which are interested in remote structure
   */
  class tProxyFrameworkElement : public core::tFrameworkElement
  {
    friend class tRemoteServer;
  private:

    // Outer class RemoteServer
    tRemoteServer* const outer_class_ptr;

    /*! Has port been found again after reconnect? */
    bool refound;

    /*! Handle in remote runtime environment */
    int remote_handle;

    /*! Is this a place-holder framework element for info that we will receive later? */
    bool yet_unknown;

  protected:

    virtual void PrepareDelete()
    {
      outer_class_ptr->remote_element_register.Remove(-remote_handle);
      ::finroc::core::tFrameworkElement::PrepareDelete();
    }

  public:

    /*! Constructor for yet anonymous element */
    tProxyFrameworkElement(tRemoteServer* const outer_class_ptr_, int handle, int extra_flags, int lock_order);

    bool Matches(const core::tFrameworkElementInfo& info);

    /*!
     * Update information about framework element
     *
     * \param info Information
     */
    void UpdateFromPortInfo(const core::tFrameworkElementInfo& info);

  };

public:

  /*!
   * Local port that acts as proxy for ports on remote machines
   */
  class tProxyPort : public tTCPPort
  {
    friend class tRemoteServer;
  private:

    // Outer class RemoteServer
    tRemoteServer* const outer_class_ptr;

    /*! Has port been found again after reconnect? */
    bool refound;

    /*! >= 0 when port has subscribed to server; value of current subscription */
    int16 subscription_strategy;

    /*! true, if current subscription includes reverse push strategy */
    bool subscription_rev_push;

    /*! Update time of current subscription */
    int16 subscription_update_time;

    /*!
     * Update port properties/information from received port information
     *
     * \param port_info Port info
     */
    void UpdateFromPortInfo(const core::tFrameworkElementInfo& port_info);

  protected:

    virtual void CheckSubscription();

    virtual void ConnectionRemoved()
    {
      CheckSubscription();
    }

    virtual void NewConnection()
    {
      CheckSubscription();
    }

    virtual void PrepareDelete();

    virtual void PropagateStrategyOverTheNet()
    {
      CheckSubscription();
    }

  public:

    /*!
     * \param port_info Port information
     */
    tProxyPort(tRemoteServer* const outer_class_ptr_, const core::tFrameworkElementInfo& port_info);

    /*!
     * Is port the one that is described by this information?
     *
     * \param info Port information
     * \return Answer
     */
    bool Matches(const core::tFrameworkElementInfo& info);

    void Reset();

  };

public:

  /*!
   * Client TCP Connection.
   *
   * This class is used on client side and
   * represents a single I/O TCP Connection with
   * own socket.
   */
  class tConnection : public tTCPConnection
  {
    friend class tRemoteServer;
  private:

    // Outer class RemoteServer
    tRemoteServer* const outer_class_ptr;

  protected:

    virtual tTCPPort* LookupPortForCallHandling(int port_index);

  public:

    /*! Command buffer for subscriptions etc. */
    //private final CoreByteBuffer commandBuffer = new CoreByteBuffer(2000, ByteOrder.BIG_ENDIAN);

    /*!
     * Client side constructor
     *
     * \param type Connection type
     */
    tConnection(tRemoteServer* const outer_class_ptr_, int8 type);

    void Connect(::std::shared_ptr<util::tNetSocket> socket_, ::std::shared_ptr<tRemoteServer::tConnection> connection);

    virtual void HandleDisconnect()
    {
      outer_class_ptr->Disconnect();
    }

    virtual void HandlePingTimeExceed()
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "TCPClient warning: critical ping time exceeded");
    }

    virtual void ProcessRequest(int8 op_code);

    virtual bool SendData(int64 start_time);

    /*!
     * Subscribe to port changes on remote server
     *
     * \param index Port index in remote runtime
     * \param strategy Strategy to use/request
     * \param update_interval Minimum interval in ms between notifications (values <= 0 mean: use server defaults)
     * \param local_index Local Port Index
     * \param data_type DataType got from server
     */
    void Subscribe(int index, int16 strategy, bool reverse_push, int16 update_interval, int local_index);

    /*!
     * Unsubscribe from port changes on remote server
     *
     * \param index Port index in remote runtime
     */
    void Unsubscribe(int index);

  };

};

} // namespace finroc
} // namespace tcp

#endif // PLUGINS__TCP__TREMOTESERVER_H
