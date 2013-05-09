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
/*!\file    plugins/tcp/internal/tNetworkPortInfo.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-02-22
 *
 * \brief   Contains tNetworkPortInfo
 *
 * \b tNetworkPortInfo
 *
 * Port annotation storing information on network ports.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__tcp__internal__tNetworkPortInfo_h__
#define __plugins__tcp__internal__tNetworkPortInfo_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tFrameworkElement.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/tcp/internal/tPeerImplementation.h"
#include "plugins/tcp/internal/tRemotePart.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace tcp
{
namespace internal
{

typedef typename core::tFrameworkElement::tHandle tHandle;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Network port information
/*!
 * Port annotation storing information on network ports.
 */
class tNetworkPortInfo : public core::tAnnotation
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tNetworkPortInfo(tRemotePart& remote_part, tHandle remote_handle, int16_t strategy, bool server_port, core::tAbstractPort& port, tHandle served_port_handle = 0);

  virtual void AnnotatedObjectToBeDeleted();

  /*!
   * Initiates subscription check in TCP thread if port is suitable and no such
   * check is already pending for this port.
   *
   * \param pending_subscription_checks Collection of pending checks. Check may be added to this queue.
   */
  inline void CheckSubscription(std::vector<tHandle>& pending_subscription_checks, rrlib::thread::tMutex& pending_subscription_checks_mutex)
  {
    core::tAbstractPort* port = GetAnnotated<core::tAbstractPort>();
    if ((!server_port) && data_ports::IsDataFlowType(port->GetDataType()) && (!subscription_check_pending.load()))
    {
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Checking subscription of ", port->GetQualifiedName());
      subscription_check_pending.store(true);
      rrlib::thread::tLock lock(pending_subscription_checks_mutex);
      pending_subscription_checks.push_back(port->GetHandle());
    }
  }

  /*!
   * Changes strategy of this (remote) port
   *
   * \param new_strategy New Strategy to set
   */
  void ChangeStrategy(int16_t new_strategy);

  /*!
   * Performs subscription check. Called by TCP thread.
   */
  void DoSubscriptionCheck();

  /*!
   * \return Last time port data was written to network
   */
  rrlib::time::tTimestamp GetLastUpdate()
  {
    return last_update;
  }

  /*!
   * \return Port handle in remote runtime environment
   */
  tHandle GetRemoteHandle() const
  {
    return remote_handle;
  }

  /*!
   * \return Remote part that port belongs to - NULL in case of server port
   */
  tRemotePart& GetRemotePart()
  {
    return remote_part;
  }

  /*!
   * \return In case this is a server port: Local handle of served port - otherwise 0
   */
  tHandle GetServedPortHandle() const
  {
    return served_port_handle;
  }

  /*!
   * \return Minimum update interval for this port
   */
  rrlib::time::tDuration GetUpdateInterval()
  {
    return std::chrono::milliseconds(40); // TODO
  }

  /*!
   * \return Is this a server port?
   */
  bool IsServerPort() const
  {
    return server_port;
  }

  void OnPortChange(data_ports::tPortDataPointer<const rrlib::rtti::tGenericObject>& value, data_ports::tChangeContext& change_context)
  {
    //FINROC_LOG_PRINT(DEBUG, "Port Changed ", port.GetWrapped()->GetQualifiedName(), " ", rrlib::serialization::Serialize(*value));
    remote_part.GetPeerImplementation().PortChanged(value, this, change_context);
  }

  /*!
   * Process incoming buffer change.
   * Buffer passed to this function was received by TCP thread.
   * Now he can add it to queue 'values_to_send' (which is only processed by TCP thread)
   */
  void ProcessIncomingBuffer(tPeerImplementation::tChangeEventPointer& change_event)
  {
    //FINROC_LOG_PRINT(DEBUG_WARNING, "xdg");
    if ((!deleted)/* && strategy > 0*/)
    {
      if ((!server_port) && values_to_send.GetMaxLength() == 0) // if we have unlucky timing, subscription is checked afterwards - so do this here as well
      {
        DoSubscriptionCheck();
      }
      if (values_to_send.GetMaxLength() > 0)
      {
        bool old_values_to_send_empty = values_to_send.Size() == 0;
        values_to_send.Enqueue(change_event);
        if (old_values_to_send_empty)
        {
          remote_part.EnqueuePortWithDataToSend(this, GetAnnotated<core::tAbstractPort>()->GetFlag(core::tFrameworkElement::tFlag::EXPRESS_PORT));
        }
      }
    }
  }

  /*!
   * Sets subscription data of server ports to specified values
   */
  void SetServerSideSubscriptionData(int16_t strategy, bool reverse_push, int16_t update_time, rrlib::serialization::tDataEncoding encoding);

  /*!
   * Writes all values to send to provided stream.
   * Resets values to send afterwards
   */
  void WriteDataBuffersToStream(rrlib::serialization::tOutputStream& stream, const rrlib::time::tTimestamp& time_now)
  {
    typename tPeerImplementation::tChangeEventPointer value_to_send = values_to_send.Dequeue();
    //FINROC_LOG_PRINT(ERROR, "a");
    if (value_to_send)
    {
      bool express_data = GetAnnotated<core::tAbstractPort>()->GetFlag(core::tFrameworkElement::tFlag::EXPRESS_PORT);
      if (express_data && values_to_send.Size() == 0)
      {
        // We only have one value to send
        if (value_to_send->new_value.GetTimestamp() == rrlib::time::cNO_TIME)
        {
          //FINROC_LOG_PRINT(WARNING, GetAnnotated<core::tAbstractPort>()->GetQualifiedName());
          tSmallPortValueChangeWithoutTimestamp::Serialize(false, stream, remote_handle, desired_encoding);
          stream << value_to_send->change_type;
          value_to_send->new_value->Serialize(stream, desired_encoding);
          stream.WriteBoolean(false);
          tSmallPortValueChangeWithoutTimestamp::FinishMessage(stream);
        }
        else
        {
          tSmallPortValueChange::Serialize(false, stream, remote_handle, desired_encoding);
          stream << value_to_send->change_type << value_to_send->new_value.GetTimestamp();
          value_to_send->new_value->Serialize(stream, desired_encoding);
          stream.WriteBoolean(false);
          tSmallPortValueChange::FinishMessage(stream);
        }
      }
      else
      {
        //FINROC_LOG_PRINT(ERROR, "a2");
        tPortValueChange::Serialize(false, stream, remote_handle, desired_encoding);
        do
        {
          stream << value_to_send->change_type << value_to_send->new_value.GetTimestamp();
          value_to_send->new_value->Serialize(stream, desired_encoding);
          value_to_send = values_to_send.Dequeue();
          stream.WriteBoolean(value_to_send.get());
        }
        while (value_to_send);
        tPortValueChange::FinishMessage(stream);
      }
    }
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Port handle in remote runtime environment */
  const tHandle remote_handle;

  /*! Is this a server port? */
  const bool server_port;

  /*! Strategy to use for this port - if it is destination port */
  int16_t strategy;

  /*! Current subscription: strategy (-1 no subscription, 0 pull subscription, 1+ push subscription with specified queue length) */
  int16_t current_subscription_strategy;

  /*! Current subscription: reverse pushing */
  bool current_subscription_reverse_push;

  /*! Current subscription: update time */
  int16_t current_subscription_update_time;

  /*! True, if subscription check is pending for this port */
  std::atomic<bool> subscription_check_pending;

  /*! Remote part that port belongs to */
  tRemotePart& remote_part;

  /*! FIFO queue with values to send to connection partners */
  rrlib::concurrent_containers::tQueue < typename tPeerImplementation::tChangeEventPointer, rrlib::concurrent_containers::tConcurrency::NONE,
        rrlib::concurrent_containers::tDequeueMode::FIFO, true > values_to_send;

  /*! Has port been deleted? (non-volatile, because ports are only deleted by TCP thread and this value is only checked by TCP thread) */
  bool deleted;

  /*! Last time port data was written to network */
  rrlib::time::tTimestamp last_update;

  /*! Desired encoding of network partner */
  rrlib::serialization::tDataEncoding desired_encoding;

  /*! In case this is a server port: Local handle of served port - otherwise 0 */
  const tHandle served_port_handle;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
