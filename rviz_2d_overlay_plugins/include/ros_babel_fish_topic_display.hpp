// -*- mode: c++; -*-
/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
 * Copyright (c) 2024, rcp1
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef JSK_RVIZ_PLUGINS_ROS_BABEL_FISH_TOPIC_DISPLAY_HPP_
#define JSK_RVIZ_PLUGINS_ROS_BABEL_FISH_TOPIC_DISPLAY_HPP_

#include <ros_babel_fish/babel_fish.hpp>
#include <rviz_common/ros_topic_display.hpp>

namespace rviz_2d_overlay_plugins {

/** @brief Display subclass using a compound ros_babel_fish::BabelFishSubscription.
 *
 * This class handles subscribing and unsubscribing to a ROS node when the display is
 * enabled or disabled. */
class RosBabelFishTopicDisplay : public rviz_common::_RosTopicDisplay
{
public:
  /** @brief Convenience typedef so subclasses don't have to change their implementation compared to
   * templated topic display. */
  typedef RosBabelFishTopicDisplay RTDClass;

  RosBabelFishTopicDisplay()
  : messages_received_(0)
  {
  }

  ~RosBabelFishTopicDisplay() override
  {
    unsubscribe();
  }

  void reset() override
  {
    Display::reset();
    messages_received_ = 0;
  }

  void setTopic(const QString & topic, const QString & datatype) override
  {
    (void) datatype;
    topic_property_->setString(topic);
  }

protected:
  void updateTopic() override
  {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  virtual void subscribe()
  {
    if (!isEnabled() ) {
      return;
    }

    if (topic_property_->isEmpty()) {
      setStatus(
        rviz_common::properties::StatusProperty::Error,
        "Topic",
        QString("Error subscribing: Empty topic name"));
      return;
    }

    if (topic_property_->getMessageType().isEmpty()) {
      setStatus(
        rviz_common::properties::StatusProperty::Error,
        "Topic",
        QString("Error subscribing: Empty topic message type"));
      return;
    }

    try {
      auto fish = ros_babel_fish::BabelFish::make_shared();
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.event_callbacks.message_lost_callback =
        [&](rclcpp::QOSMessageLostInfo & info)
        {
          std::ostringstream sstm;
          sstm << "Some messages were lost:\n>\tNumber of new lost messages: " <<
            info.total_count_change << " \n>\tTotal number of messages lost: " <<
            info.total_count;
          setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", QString(sstm.str().c_str()));
        };

      rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();
      try {
        subscription_ =
          fish->create_subscription(
          *node,
          topic_property_->getTopicStd(),
          topic_property_->getMessageType().toStdString(),
          qos_profile,
          [this](ros_babel_fish::CompoundMessage::ConstSharedPtr message) {incomingMessage(message);},
          nullptr,
          sub_opts);
        subscription_start_time_ = node->now();
        setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
      } catch (ros_babel_fish::BabelFishException & e) {
        setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
      }
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe()
  {
    subscription_.reset();
  }

  void onEnable() override
  {
    subscribe();
  }

  void onDisable() override
  {
    unsubscribe();
    reset();
  }

  void fixedFrameChanged() override
  {
    reset();
  }

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessage(ros_babel_fish::CompoundMessage::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }

    ++messages_received_;
    QString topic_str = QString::number(messages_received_) + " messages received";
    // Append topic subscription frequency if we can lock rviz_ros_node_.
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_interface =
      rviz_ros_node_.lock();
    if (node_interface != nullptr) {
      const double duration =
        (node_interface->get_raw_node()->now() - subscription_start_time_).seconds();
      const double subscription_frequency =
        static_cast<double>(messages_received_) / duration;
      topic_str += " at " + QString::number(subscription_frequency, 'f', 1) + " hz.";
    }
    setStatus(
      rviz_common::properties::StatusProperty::Ok,
      "Topic",
      topic_str);

    processMessage(msg);
  }

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessage(const ros_babel_fish::CompoundMessage::ConstSharedPtr msg) = 0;

  ros_babel_fish::BabelFishSubscription::SharedPtr subscription_;
  rclcpp::Time subscription_start_time_;
  uint32_t messages_received_;
};

}  // namespace rviz_2d_overlay_plugins

#endif // JSK_RVIZ_PLUGINS_ROS_BABEL_FISH_TOPIC_DISPLAY_HPP_
