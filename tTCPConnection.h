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

#ifndef plugins__tcp__tTCPConnection_h__
#define plugins__tcp__tTCPConnection_h__

#include <array>
#include "rrlib/finroc_core_utils/definitions.h"
#include "rrlib/finroc_core_utils/tAtomicDoubleInt.h"
#include "rrlib/finroc_core_utils/container/tWonderQueueUniquePtr.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "rrlib/finroc_core_utils/net/tNetSocket.h"
#include "rrlib/serialization/serialization.h"
#include "rrlib/rtti/tDataTypeBase.h"

#include "core/parameter/tParameterNumeric.h"
#include "core/port/net/tRemoteTypes.h"
#include "core/tLockOrderLevels.h"
#include "core/port/net/tUpdateTimeChangeListener.h"
#include "core/thread/tCoreLoopThreadBase.h"

#include "plugins/tcp/tTCPPort.h"
#include "plugins/tcp/tTCPSettings.h"
#include "plugins/tcp/tTCP.h"

namespace finroc
{
namespace tcp
{
class tTCPPeer;

/*!
 * \author Max Reichardt
 *
 * Common parts of client and server TCP Connections
 *
 * (writer and listener members need to be initialized by subclass)
 */
class tTCPConnection : public util::tLogUser, public core::tUpdateTimeChangeListener, public rrlib::thread::tRecursiveMutex
{
public:

  /*!
   * Listens at socket for incoming data
   */
  class tReader : public core::tCoreLoopThreadBase
  {
  private:

    // Outer class TCPConnection
    tTCPConnection& outer_class;

    /*!
     * Check that command is terminated correctly when TCPSettings.DEBUG_TCP is activated
     */
    void CheckCommandEnd();

  public:

    tReader(tTCPConnection& outer_class, const util::tString& description);

    virtual void MainLoopCallback()
    {
    }

    virtual void Run();

    virtual void StopThread();

  };

public:

  /*!
   * Writes outgoing data to socket
   */
  class tWriter : public core::tCoreLoopThreadBase
  {
    friend class tTCPConnection;
  private:

    // Outer class TCPConnection
    tTCPConnection& outer_class;

    /*! Index of last packet that was acknowledged */
    int last_ack_index;

    /*! Index of currently sent packet */
    volatile int cur_packet_index;

    /*! State at this change counter state are/were handled */
    int handled_changed_counter;

    /*!
     * Double int to efficiently handle when to lay writer thread to sleep
     * [1 bit sleeping?][30 bit change counter]
     */
    util::tAtomicDoubleInt writer_synch;

    /*! Queue with calls/commands waiting to be sent */
    util::tWonderQueueUniquePtr<core::tSerializableReusable, core::tSerializableReusable::tRecycler> calls_to_send;

  public:

    tWriter(tTCPConnection& outer_class, const util::tString& description);

    bool CanSend();

    virtual ~tWriter();

    virtual void MainLoopCallback()
    {
    }

    /*!
     * Notify (possibly wake-up) writer thread. Should be called whenever new tasks for the writer arrive.
     */
    void NotifyWriter();

    virtual void Run();

    /*!
     * Send answers for any pending acknowledgement requests and commands like subscriptions
     */
    void SendAcknowledgementsAndCommands();

    /*!
     * Send call to connection partner
     *
     * \param call Call object
     */
    void SendCall(core::tSerializableReusable::tPtr& call);

    virtual void StopThread();

  };

  friend class tTCPPort;
private:

  /*! References to Connection parameters */
  core::tParameter<rrlib::time::tDuration>& min_update_interval;

  core::tParameter<int>& max_not_acknowledged_packets;

  /*! Index of last acknowledged sent packet */
  std::atomic<int> last_acknowledged_packet;

  /*! Index of last acknowledgement request that was received */
  std::atomic<int> last_ack_request_index;

  /*! Timestamp of when packet n was sent (Index is n % MAX_NOT_ACKNOWLEDGED_PACKETS => efficient and safe implementation (ring queue)) */
  std::array < rrlib::time::tTimestamp, tTCPSettings::cMAX_NOT_ACKNOWLEDGED_PACKETS + 1 > sent_packet_time;

  /*! Ping time for last packages (Index is n % AVG_PING_PACKETS => efficient and safe implementation (ring queue)) */
  std::array < rrlib::time::tDuration, tTCPSettings::cAVG_PING_PACKETS + 1 > ping_times;

  /*! Ping time statistics */
  rrlib::time::tAtomicDuration avg_ping_time, max_ping_time;

  /*! Signal for disconnecting */
  std::atomic<bool> disconnect_signal;

protected:

  /*! Network Socket used for accessing remote Server */
  std::shared_ptr<util::tNetSocket> socket;

  /*! Output Stream for sending data to remote Server */
  std::shared_ptr<rrlib::serialization::tOutputStream> cos;

  /*! Input Stream for receiving data ro remote Server */
  std::shared_ptr<rrlib::serialization::tInputStream> cis;

  /*! Listener Thread */
  //protected @SharedPtr Reader listener;

  /*! Writer Thread */
  std::weak_ptr<tWriter> writer;

  /*! Reader Thread */
  std::weak_ptr<tReader> reader;

  /*! Timestamp relative to which time is encoded in this stream */
  rrlib::time::tTimestamp time_base;

  /*! default connection times of connection partner */
  std::shared_ptr<core::tRemoteTypes> update_times;

  /*! Connection type - BULK or EXPRESS */
  int8 type;

  /*! Ports that are monitored for changes by this connection and should be checked for modifications */
  util::tSafeConcurrentlyIterableList<tTCPPort*, rrlib::thread::tOrderedMutex> monitored_ports;

  /*! TCPPeer that this connection belongs to (null if it does not belong to a peer) */
  tTCPPeer* peer;

  /*! Send information about peers to partner port? */
  bool send_peer_info_to_partner;

  /*! Last version of peer information that was sent to connection partner */
  int last_peer_info_sent_revision;

  /*! Rx related: last time RX was retrieved */
  rrlib::time::tTimestamp last_rx_timestamp;

  /*! Rx related: last time RX was retrieved: how much have we received in total? */
  int64_t last_rx_position;

private:

  /*!
   * Updates ping statistic variables
   */
  void UpdatePingStatistics();

protected:

  /*!
   * Handles method calls on server and client side
   */
  void HandleMethodCall();

  /*!
   * Handles returning method calls on server and client side
   */
  void HandleMethodCallReturn();

  /*!
   * Handles pull calls on server and client side
   */
  void HandlePullCall();

  /*!
   * Handles returning pull call on server and client side
   */
  void HandleReturningPullCall();

  /*!
   * When calls are received: Lookup port responsible for handling that call
   *
   * \param port_index Port index as received from network stream
   */
  virtual tTCPPort* LookupPortForCallHandling(int port_index) = 0;

public:

  /*!
   * \param type Connection type
   * \param peer TCPPeer that this connection belongs to (null if it does not belong to a peer)
   * \param send_peer_info_to_partner Send information about peers to partner port?
   */
  tTCPConnection(int8 type_, tTCPPeer* peer_, bool send_peer_info_to_partner_);

  virtual ~tTCPConnection();

  /*!
   * Should be called regularly by monitoring thread to check whether critical
   * ping time threshold is exceeded.
   *
   * \return Time the calling thread may wait before calling again (it futile to call this method before)
   */
  rrlib::time::tDuration CheckPingForDisconnect();

  /*!
   * Close connection
   */
  void Disconnect();

  /*!
   * \return Is TCP connection disconnecting?
   */
  inline bool Disconnecting()
  {
    return disconnect_signal.load();
  }

  /*!
   * \return Average ping time among last TCPSettings.AVG_PING_PACKETS packets
   */
  inline rrlib::time::tDuration GetAvgPingTime()
  {
    return avg_ping_time.Load();
  }

  /*!
   * \return Type of connection ("Bulk" oder "Express")
   */
  inline util::tString GetConnectionTypeString()
  {
    return (type == tTCP::cTCP_P2P_ID_BULK ? "Bulk" : "Express");
  }

  /*!
   * \return Maximum ping time among last TCPSettings.AVG_PING_PACKETS packets
   */
  inline rrlib::time::tDuration GetMaxPingTime()
  {
    return max_ping_time.Load();
  }

  /*!
   * \return Data rate of bytes read from network (in bytes/s)
   */
  int GetRx();

  /*!
   * Called when listener or writer is disconnected
   */
  virtual void HandleDisconnect() = 0;

  /*!
   * Called when cricical ping time threshold was exceeded
   */
  virtual void HandlePingTimeExceed() = 0;

  /*!
   * Notify (possibly wake-up) writer thread. Should be called whenever new tasks for the writer arrive.
   */
  void NotifyWriter();

  /*!
   * \return Is critical ping time currently exceeded (possibly temporary disconnect)
   */
  bool PingTimeExceeed();

  /*!
   * Called when listener receives request
   */
  virtual void ProcessRequest(tOpCode op_code) = 0;

//  inline int64 ReadTimestamp()
//  {
//    return cis->ReadInt() + time_base;
//  }

  /*!
   * Send call to connection partner
   *
   * \param call Call object
   */
  void SendCall(core::tSerializableReusable::tPtr call);

  /*!
   * Send data chunk. Is called regularly in writer loop whenever changed flag is set.
   *
   * \param start_time Timestamp when send operation was started
   * \return Is this an packet that needs acknowledgement ?
   */
  virtual bool SendData(const rrlib::time::tTimestamp& start_time) = 0;

  /*!
   * Common/standard implementation of above
   *
   * \param start_time Timestamp when send operation was started
   * \param op_code OpCode to use for send operations
   * \return Is this an packet that needs acknowledgement ?
   */
  bool SendDataPrototype(const rrlib::time::tTimestamp& start_time, tOpCode op_code);

  /*!
   * Needs to be called after a command has been serialized to the output stream
   */
  void TerminateCommand();

  virtual void UpdateTimeChanged(rrlib::rtti::tDataTypeBase dt, int16 new_update_time);

//  inline void WriteTimestamp(int64 time_stamp)
//  {
//    cos->WriteInt(static_cast<int>((time_stamp - time_base)));
//  }

};

} // namespace finroc
} // namespace tcp

#endif // plugins__tcp__tTCPConnection_h__
