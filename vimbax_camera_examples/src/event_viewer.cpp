/*
 * Copyright (c) 2024, Allied Vision Technologies GmbH
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

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_events/event_subscriber.hpp>

#include <vimbax_camera_msgs/msg/event_data.hpp>

#include "example_helper.hpp"

int main(int argc, char * argv[])
{
  auto const args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (args.size() < 3) {
    std::cerr << "Usage: " + args[0] + " <node namespace> <event name>" << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("_event_viewer");

  auto topic = build_topic_path(args[1], "/events");

  auto event_subscriber =
    vimbax_camera_events::EventSubscriber<vimbax_camera_msgs::msg::EventData>::make_shared(
    node,
    topic
    );


  auto event_subscription = event_subscriber->subscribe_event(
    args[2], [&](auto event_data) {
      RCLCPP_INFO(node->get_logger(), "Got event meta data:");
      for (auto const & entry : event_data.entries) {
        RCLCPP_INFO(node->get_logger(), "%s: %s", entry.name.c_str(), entry.value.c_str());
      }
    });

  std::thread spin_thread([node] {
      rclcpp::spin(node);
    });

  try {
    auto subscription = event_subscription.get();
  } catch (std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), ex.what());
    rclcpp::shutdown();
  }


  spin_thread.join();

  return 0;
}
