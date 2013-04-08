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
/*!\file    plugins/tcp/internal/tRemotePart.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-01-04
 *
 * \brief   Contains tRemotePart
 *
 * \b tRemotePart
 *
 * Class that represent a remote runtime environment.
 * It creates a proxy port for each shared port in the remote runtime.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tRemotePart_h__
#define __plugins__tcp__internal__tRemotePart_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tFrameworkElement.h"
#include "plugins/data_ports/tPullRequestHandler.h"
#include "plugins/rpc_ports/internal/tRPCPort.h"
#include "plugins/rpc_ports/internal/tResponseSender.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tConnection.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace tcp
{
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
class tPeerInfo;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Remote part
/*!
 * Class that represent a remote runtime environment.
 * It creates a proxy port for each shared port in the remote runtime.
 */
class tRemotePart : public core::tFrameworkElement, public rpc_ports::internal::tResponseSender, public data_ports::tPullRequestHandler<rrlib::rtti::tGenericObject>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename rpc_ports::internal::tRPCPort::tCallPointer tCallPointer;

  /*! Peer info this part is associated with */
  tRemotePart(tPeerInfo& peer_info, core::tFrameworkElement& parent, tPeerImplementation& peer_implementation);

  ~tRemotePart();

  /*!
   * Add connection for this remote part
   *
   * \param connection Connection to add (with flags set)
   * \return Did this succeed? (fails if there already is a connection for specified types of data; may happen if two parts try to connect at the same time - only one connection is kept)
   */
  bool AddConnection(std::shared_ptr<tConnection> connection);

  /*!
   * Add remote port specified by provided info to this remote part
   */
  void AddRemotePort(common::tFrameworkElementInfo& info);

  /*!
   * Enqueue port with data to send.
   * TCP thread will send data in this port in SendPendingMessages() function
   */
  void EnqueuePortWithDataToSend(tNetworkPortInfo* network_port_info, bool express_data)
  {
    std::vector<tNetworkPortInfo*>& vector = express_data ? ports_with_express_data_to_send : ports_with_bulk_data_to_send;
    vector.push_back(network_port_info);
  }

  /*!
   * \return Structure information to send to remote part
   */
  common::tStructureExchange GetDesiredStructureInfo() const
  {
    return send_structure_info;
  }

  /*!
   * \return Bulk connection
   */
  std::shared_ptr<tConnection> GetBulkConnection()
  {
    return bulk_connection;
  }

  /*!
   * \return Express connection
   */
  std::shared_ptr<tConnection> GetExpressConnection()
  {
    return express_connection;
  }

  /*!
   * \return Framework element that contains all global links. Is created if it does not exist.
   */
  tFrameworkElement* GetGlobalLinksElement();

  /*!
   * \return Peer implementation that this remote part belongs to
   */
  tPeerImplementation& GetPeerImplementation()
  {
    return peer_implementation;
  }

  /*!
   * \return Framework element that contains all server ports. Is created if it does not exist.
   */
  tFrameworkElement* GetServerPortsElement();

  /*!
   * \return Management connection
   */
  std::shared_ptr<tConnection> GetManagementConnection()
  {
    return management_connection;
  }

  /*!
   * Called when network port is deleted.
   * Will remove port from ports_with_express_data_to_send and ports_with_bulk_data_to_send
   */
  void PortDeleted(tNetworkPortInfo* deleted_port);

  /*!
   * \return Ports that currently have bulk data to send
   */
  std::vector<tNetworkPortInfo*>& PortsWithBulkDataToSend()
  {
    return ports_with_bulk_data_to_send;
  }

  /*!
   * \return Ports that currently have express data to send
   */
  std::vector<tNetworkPortInfo*>& PortsWithExpressDataToSend()
  {
    return ports_with_express_data_to_send;
  }

  /*!
   * Process message with specified opcode in provided memory buffer
   *
   * \return Defer message? (in this case, ProcessMessage() will be called again later with the same message)
   */
  bool ProcessMessage(tOpCode opcode, rrlib::serialization::tMemoryBuffer& buffer, common::tRemoteTypes& remote_types, tConnection& connection);

  /*!
   * Processes remote structure information/changes
   *
   * \param stream Input stream on packet
   */
  void ProcessStructurePacket(rrlib::serialization::tInputStream& stream);

  /*!
   * Removes connection for this remote part
   *
   * \param connection Connection to remove
   */
  void RemoveConnection(tConnection& connection);

  /*!
   * Called by TCP Thread whenever rpc ports were deleted
   *
   * \param deleted_ports List with handles of deleted RPC ports
   */
  void RpcPortsDeleted(std::vector<core::tFrameworkElement::tHandle>& deleted_ports);

  /*!
   * Sends specified RPC call to connection partner
   */
  void SendCall(tCallPointer& call_to_send, const rrlib::time::tTimestamp& time_now);

  /*!
   * Sends pending messages in all connections
   */
  void SendPendingMessages(const rrlib::time::tTimestamp& time_now);

  /*!
   * Sends structure change to remote part
   *
   * \param structure_change Serialized structure change to send
   */
  inline void SendStructureChange(const tSerializedStructureChange& structure_change)
  {
    if (management_connection)
    {
      management_connection->SendStructureChange(structure_change, send_structure_info);
    }
  }

  /*!
   * \param send_structure_info Structure information to send to remote part
   */
  void SetDesiredStructureInfo(common::tStructureExchange send_structure_info);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*!
   * Stores information on pull call to be answered by connection partner
   */
  struct tPullCallInfo
  {
    /*! Call id of pull call */
    rpc_ports::internal::tCallId call_id;

    /*! Promise for waiting thread */
    std::shared_ptr<rpc_ports::tPromise<data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject>>> promise;

    /*! Handle of local port that requested pull operation */
    tFrameworkElementHandle local_port_handle;

    /*! Handle of remote port that requested pull operation */
    tFrameworkElementHandle remote_port_handle;

    /*! Time when call times out */
    rrlib::time::tTimestamp timeout_time;

    /*! Pointer to remote part */
    tRemotePart* remote_part;


    tPullCallInfo(tRemotePart& remote_part, data_ports::common::tAbstractDataPort& local_port);

    /*! Used as boost asio handler to forward this pull call to TCP Thread */
    void operator()();
  };

  friend struct tPullCallInfo;

  /*! Peer info that this part belongs to */
  tPeerInfo& peer_info;

  /*! Peer implementation that this remote part belongs to */
  tPeerImplementation& peer_implementation;

  /*! Connection to transfer "express" ports data */
  std::shared_ptr<tConnection> express_connection;

  /*! Connection to transfer "bulk" ports data */
  std::shared_ptr<tConnection> bulk_connection;

  /*! Connection to transfer all other "management" data */
  std::shared_ptr<tConnection> management_connection;

  /*! Structure information to send to remote part */
  common::tStructureExchange send_structure_info;

  /*! Framework element that contains all global links - possibly NULL */
  core::tFrameworkElement* global_links;

  /*! Framework element that contains all server ports - possibly NULL */
  core::tFrameworkElement* server_ports;

  /*! Maps local port handle => server port */
  std::map<tFrameworkElementHandle, data_ports::tGenericPort> server_port_map;

  /*! Maps remote port handler => replicated port */
  std::map<tFrameworkElementHandle, core::tAbstractPort*> remote_port_map;

  /*! Ports that currently have express and bulk data to send */
  std::vector<tNetworkPortInfo*> ports_with_express_data_to_send, ports_with_bulk_data_to_send;

  /*! Calls that were not ready for sending yet */
  std::vector<tCallPointer> not_ready_calls;

  /*! Calls that wait for a response */
  std::vector<std::pair<rrlib::time::tTimestamp, tCallPointer>> calls_awaiting_response;

  /*! Next call id to assign to sent call */
  rpc_ports::internal::tCallId next_call_id;

  /*! Pull calls that wait for a response */
  std::vector<tPullCallInfo> pull_calls_awaiting_response;


  virtual data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject> PullRequest(data_ports::common::tAbstractDataPort& origin); // TODO: mark override in gcc 4.7

  /*!
   * Sends specified RPC call to connection partner
   *
   * \param call_to_send Call to send (already checked whether it is ready)
   */
  void SendCallImplementation(tCallPointer& call_to_send, const rrlib::time::tTimestamp& time_now);

  /*!
   * Sends pull request to remote peer
   * (May only be called by TCP Thread)
   */
  void SendPullRequest(tPullCallInfo& pull_call_info);

  virtual void SendResponse(typename tResponseSender::tCallPointer && response_to_send); // TODO: mark override in gcc 4.7
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
