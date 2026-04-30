// Copyright 2022 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <chrono>
#include <thread>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#if __has_include(<hardware_interface/hardware_interface/version.h>)
#include <hardware_interface/hardware_interface/version.h>
#else
#include <hardware_interface/version.h>
#endif
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/descriptions.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <topic_based_ros2_control/topic_based_system.hpp>

TEST(TestTopicBasedSystem, load_topic_based_system_2dof)
{
  const std::string hardware_system_2dof_standard_interfaces_with_topic_based =
      R"(
  <ros2_control name="TopicBasedSystem2dof" type="system">
    <hardware>
      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
      <param name="joint_commands_topic">/topic_based_joint_commands</param>
      <param name="joint_states_topic">/topic_based_custom_joint_states</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_standard_interfaces_with_topic_based +
              ros2_control_test_assets::urdf_tail;
  auto node = std::make_shared<rclcpp::Node>("test_topic_based_system");

// The API of the RessourceManager has changed in hardware_interface 4.13.0
#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf, node->get_node_clock_interface(),
                                                         node->get_node_logging_interface(), false));
#else
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf, true, false));
#endif
}

TEST(TestTopicBasedSystem, write_omits_state_only_joints_from_command_message)
{
  const std::string hardware_system_with_passive_joint = R"(
  <ros2_control name="TopicBasedSystemPassiveJoint" type="system">
    <hardware>
      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
      <param name="joint_commands_topic">/topic_based_joint_commands_regression</param>
      <param name="joint_states_topic">/topic_based_custom_joint_states_regression</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";

  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_with_passive_joint +
              ros2_control_test_assets::urdf_tail;
  const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);
  ASSERT_EQ(hardware_info.size(), 1u);

  topic_based_ros2_control::TopicBasedSystem system;
  ASSERT_EQ(system.on_init(hardware_info.front()), topic_based_ros2_control::CallbackReturn::SUCCESS);

  auto command_interfaces = system.export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 1u);
  command_interfaces.front().set_value(0.42);

  auto subscriber_node = std::make_shared<rclcpp::Node>("topic_based_system_regression_subscriber");
  std::optional<sensor_msgs::msg::JointState> received_joint_state;
  const auto subscription = subscriber_node->create_subscription<sensor_msgs::msg::JointState>(
      "/topic_based_joint_commands_regression", rclcpp::QoS(1),
      [&received_joint_state](sensor_msgs::msg::JointState::SharedPtr msg) { received_joint_state = *msg; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(subscriber_node);

  // Give discovery a short window before publishing.
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < discovery_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            hardware_interface::return_type::OK);

  const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received_joint_state.has_value() && std::chrono::steady_clock::now() < receive_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(received_joint_state.has_value());
  EXPECT_EQ(received_joint_state->name, std::vector<std::string>({ "joint1" }));
  EXPECT_EQ(received_joint_state->position.size(), 1u);
  EXPECT_NEAR(received_joint_state->position.front(), 0.42, 1e-9);
  EXPECT_TRUE(received_joint_state->velocity.empty());
  EXPECT_TRUE(received_joint_state->effort.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
