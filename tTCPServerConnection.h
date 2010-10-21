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

#ifndef PLUGINS__TCP__TTCPSERVERCONNECTION_H
#define PLUGINS__TCP__TTCPSERVERCONNECTION_H

#include "rrlib/finroc_core_utils/stream/tChunkedBuffer.h"
#include "core/buffers/tCoreOutput.h"
#include "core/buffers/tCoreInput.h"
#include "core/tFrameworkElementTreeFilter.h"
#include "rrlib/finroc_core_utils/net/tNetSocket.h"
#include "core/port/tPortCreationInfo.h"
#include "plugins/tcp/tTCPConnection.h"
#include "core/tRuntimeListener.h"
#include "core/tChildIterator.h"
#include "core/tFrameworkElement.h"
#include "plugins/tcp/tTCPPort.h"
#include "core/thread/tCoreLoopThreadBase.h"

namespace finroc
{
namespace core
{
class tAbstractPort;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace tcp
{
class tTCPServer;
class tTCPPeer;

/*!
 * \author Max Reichardt
 *
 * Single connection to TCP Server
 *
 * (memory management: Should be created with new - deletes itself:
 *  Port set, as well as reader and writer threads hold shared_ptr to this connection object)
 *
 * Thread-safety: Reader thread is the only one that deletes ports while operating. So it can use them without lock.
 */
class tTCPServerConnection : public tTCPConnection, public core::tRuntimeListener, public core::tFrameworkElementTreeFilter::tCallback
{
public:
  class tPortSet; // inner class forward declaration
public:
  class tServerPort; // inner class forward declaration
public:
  class tPingTimeMonitor; // inner class forward declaration
private:

  /*! Used for creating connection IDs */
  static util::tAtomicInt connection_id;

  /*! FrameworkElement representation of this Connection (so temporary ports are grouped and can conveniently be deleted) */
  tPortSet* port_set;

  /*! Unique ids for ports in this connection */
  //private final AtomicInt portId = new AtomicInt();

  ///** lookup table for proxy ports */
  //private final int[] proxyPortLookup = new int[CoreRegister.MAX_ELEMENTS];

  /*! Send information about runtime in this connection? */
  bool send_runtime_info;

  /*!
   * Buffer for storing serialized updated runtime information - ready to be sent -
   * (efficient & completely non-blocking solution :-) )
   */
  util::tChunkedBuffer runtime_info_buffer;

  /*! For any thread that writes runtime changes - note that when declared in this order, writer will be deleted/closed before runtimeInfoBuffer (that's intended and the correct order) */
  core::tCoreOutput runtime_info_writer;

  /*! For network writer thread that forwards runtime change information */
  core::tCoreInput runtime_info_reader;

  /*! Framework element filter to decide which data is interesting for client */
  core::tFrameworkElementTreeFilter element_filter;

  /*! Temporary string builder - only used by reader thread */
  util::tStringBuilder tmp;

  /*! Number of times disconnect was called, since last connect */
  util::tAtomicInt disconnect_calls;

public:

  /*! List with connections for TCP servers in this runtime */
  static util::tSafeConcurrentlyIterableList<tTCPServerConnection*> connections;

private:

  /*!
   * (belongs to ServerPort)
   * Create matching port creation info to access specified port.
   *
   * \param counter_part Port that will be accessed
   * \return Port Creation Info
   */
  core::tPortCreationInfo InitPci(core::tAbstractPort* counter_part);

protected:

  virtual tTCPPort* LookupPortForCallHandling(int port_handle)
  {
    return GetPort(port_handle, true);
  }

public:

  /*!
   * \param s Socket with new connection
   * \param stream_id Stream ID for connection type (see TCP class)
   * \param peer Peer that this server belongs to
   */
  tTCPServerConnection(::std::tr1::shared_ptr<util::tNetSocket> s, int8 stream_id, tTCPServer* server, tTCPPeer* peer);

  /*!
   * Get Port for this connection. Creates Port if not yet existent.
   * (should only be called by reader thread with possiblyCreate=true in order to ensure thread-safety)
   *
   * \param handle Port Handle
   * \param possibly_create Possibly create network port if it does not exist
   * \return Port. Null, if it is not existent.
   */
  tServerPort* GetPort(int handle, bool possibly_create);

  virtual void HandleDisconnect();

  virtual void HandlePingTimeExceed()
  {
    port_set->NotifyPortsOfDisconnect();
  }

  virtual void ProcessRequest(int8 op_code);

  virtual void RuntimeChange(int8 change_type, core::tFrameworkElement* element);

  virtual void RuntimeEdgeChange(int8 change_type, core::tAbstractPort* source, core::tAbstractPort* target);

  virtual bool SendData(int64 start_time);

  void SerializeRuntimeChange(int8 change_type, core::tFrameworkElement* element);

  void TreeFilterCallback(core::tFrameworkElement* fe);

public:

  /*!
   * PortSet representation of this Connection (so temporary ports are grouped and can conveniently be deleted)
   */
  class tPortSet : public core::tFrameworkElement
  {
    friend class tTCPServerConnection;
  private:

    // Outer class TCPServerConnection
    tTCPServerConnection* const outer_class_ptr;

    /*! For iterating over portSet's ports */
    core::tChildIterator port_iterator;

    /*! Ensures that connection object exists as long as port set does */
    ::std::tr1::shared_ptr<tTCPServerConnection> connection_lock;

    /*!
     * Notifies ports that connection is bad/disconnected so that
     * they can apply default values if needed.
     */
    void NotifyPortsOfDisconnect();

  protected:

    virtual void PrepareDelete();

  public:

    tPortSet(tTCPServerConnection* const outer_class_ptr_, tTCPServer* server, ::std::tr1::shared_ptr<tTCPServerConnection> connection_lock_);

    //      @Override
    //      public void handleCallReturn(AbstractCall pc) {
    //          pc.setRemotePortHandle(pc.popCaller());
    //          TCPServerConnection.this.sendCall(pc);
    //      }

  };

public:

  /*!
   * Local Port that is created for subscriptions and incoming connections.
   */
  class tServerPort : public tTCPPort
  {
    friend class tTCPServerConnection;
  private:

    // Outer class TCPServerConnection
    tTCPServerConnection* const outer_class_ptr;

    /*! Local partner port */
    core::tAbstractPort* local_port;

  protected:

    virtual void PostChildInit();

    //      /**
    //       * Set Push strategy
    //       *
    //       * \param b on or off?
    //       */
    //      public void setPush(boolean b) {
    //          if (getPort().isOutputPort()) {
    //              getPort().setReversePushStrategy(b);
    //          } else {
    //              getPort().setPushStrategy(b);
    //          }
    //      }

    //      /**
    //       * \param queueLength maximum Queue Length
    //       */
    //      public void setQueueLength(int queueLength) {
    //          if (getPort().getFlag(PortFlags.OUTPUT_PORT | PortFlags.HAS_QUEUE)) {
    //              getPort().setMaxQueueLength(queueLength);
    //          }
    //      }

    //      /**
    //       * \return Current Minimum network update interval for this port (takes local and remote settings into account)
    //       */
    //      public short getMinNetUpdateInterval() {
    //          short t = 0;
    //          if ((t = updateIntervalPartner) >= 0) {
    //              return t;
    //          } else if ((t = localPort.getMinNetUpdateInterval()) >= 0) {
    //              return t;
    //          } else if ((t = updateTimes.getTime(getPort().getDataType())) >= 0) {
    //              return t;
    //          } else if ((t = getPort().getDataType().getUpdateTime()) >= 0) {
    //              return t;
    //          }
    //          return updateTimes.getGlobalDefault();
    //      }
    //
    //      @Override
    //      protected void portChanged() {
    //          notifyWriter();
    //      }

    virtual void PropagateStrategyOverTheNet()
    {
      // data is propagated automatically with port strategy changed in framework element class
    }

  public:

    /*! Edge to connect server port with local port
     * \param port_set */
    tServerPort(tTCPServerConnection* const outer_class_ptr_, core::tAbstractPort* counter_part, tTCPServerConnection::tPortSet* port_set);

    // notify any connected input ports about disconnect
    virtual void NotifyDisconnect();

  };

public:

  /*!
   * Monitors connections for critical ping time exceed
   */
  class tPingTimeMonitor : public core::tCoreLoopThreadBase
  {
    friend class tTCPServerConnection;
  private:

    static ::std::tr1::shared_ptr<tTCPServerConnection::tPingTimeMonitor> instance;

    /*! Locked before thread list (in C++) */
    static util::tMutexLockOrder static_class_mutex;

    tPingTimeMonitor();

    static tTCPServerConnection::tPingTimeMonitor* GetInstance();

  public:

    virtual void MainLoopCallback();

  };

};

} // namespace finroc
} // namespace tcp

#endif // PLUGINS__TCP__TTCPSERVERCONNECTION_H
