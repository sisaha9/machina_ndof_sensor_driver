// Copyright 2023 Siddharth Saha.
//
// Licensed under the GNU Affero General Public License, v3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/agpl-3.0.html
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: AGPL-3.0-only

#include "machina_ndof_sensor_driver_client/machina_ndof_sensor_driver_client_node.hpp"

namespace machina {
namespace client {
namespace ndof_sensor_driver {
/******************************************************************************/
MachinaNDofSensorDriverClientNode::MachinaNDofSensorDriverClientNode(
    const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("machina_ndof_sensor_driver_client_node",
                                      options) {
  // Set up the parameter listener
  param_listener_ = std::make_unique<ros_parameters::ParamListener>(
      this->get_node_parameters_interface());
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Client Node Constructed");
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverClientNode::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Retrieve the parameters. Right now all parameters are frozen on startup
  // so there is no need to call this function again
  ros_params_ = param_listener_->get_params();
  // Check if the number of sensors and the number of elements in client params
  // match
  if (ros_params_.num_sensors !=
      static_cast<int64_t>(ros_params_.collection_intervals.size())) {
    RCLCPP_ERROR(get_logger(),
                 "Number of sensors and collection intervals do not match");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }
  if (ros_params_.num_sensors !=
      static_cast<int64_t>(ros_params_.service_timeout_intervals.size())) {
    RCLCPP_ERROR(
        get_logger(),
        "Number of sensors and service timeout intervals do not match");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }
  if (ros_params_.num_sensors !=
      static_cast<int64_t>(
          ros_params_.max_service_connection_attempts.size())) {
    RCLCPP_ERROR(
        get_logger(),
        "Number of sensors and max service connection attempts do not match");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }
  // Set up the sensor clients
  sensor_output_msg_.sensors.reserve(ros_params_.num_sensors);
  for (int64_t i = 0; i < ros_params_.num_sensors; i++) {
    sensor_output_msg_.sensors.push_back(
        machina_ndof_sensor_driver_msgs::msg::SensorOutput());
    sensor_clients_.push_back(std::make_unique<MachinaNDofSensorDriverClient>(
        this, i, ros_params_.collection_intervals[i],
        ros_params_.service_timeout_intervals[i],
        ros_params_.max_service_connection_attempts[i]));
  }
  // Set up the publisher
  sensor_output_publisher_ =
      create_publisher<machina_ndof_sensor_driver_msgs::msg::JointSensorOutput>(
          "joint_sensor_output", rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Client Node Configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverClientNode::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Start the sensor clients
  for (auto& sensor_client : sensor_clients_) {
    sensor_client->StartReadingSensor();
  }
  // Activate the publisher
  sensor_output_publisher_->on_activate();
  // Start the timer
  timer_time_ = now();
  // Create a callback group for the timer so that it can run in parallel with
  // the client callbacks
  timer_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Create the timer with the callback group
  sensor_output_timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_nanoseconds(ros_params_.publish_interval),
      std::bind(&MachinaNDofSensorDriverClientNode::PublishLatestSensorData,
                this),
      timer_callback_group_);
  RCLCPP_INFO(get_logger(), "Machina NDof Sensor Driver Client Node Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverClientNode::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Deactivate the timer
  sensor_output_timer_->cancel();
  sensor_output_timer_.reset();
  // Deactivate the publisher
  sensor_output_publisher_->on_deactivate();
  // Stop the sensor clients
  for (auto& sensor_client : sensor_clients_) {
    sensor_client->StopReadingSensor();
  }
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Client Node Deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverClientNode::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Call the cleanup method
  on_cleanup(previous_state);
  RCLCPP_INFO(get_logger(), "Machina NDof Sensor Driver Client Node Shut Down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverClientNode::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Set all pointers to null
  sensor_output_timer_.reset();
  sensor_output_publisher_.reset();
  for (auto& sensor_client : sensor_clients_) {
    sensor_client.reset();
  }
  // Clear the vector
  sensor_clients_.clear();
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Client Node Cleaned Up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverClientNode::on_error(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Call the shutdown method
  on_shutdown(previous_state);
  RCLCPP_INFO(get_logger(), "Machina NDof Sensor Driver Client Node Errored");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
void MachinaNDofSensorDriverClientNode::PublishLatestSensorData() {
  // Store the time callback was called
  auto new_timer_time = now();
  // Ideally this should be in a telemetry. This checks how long it takes to
  // b/w two calls of the timer
  RCLCPP_DEBUG(get_logger(), "Took %f seconds to publish",
               (new_timer_time - timer_time_).seconds());
  // Update the time
  timer_time_ = new_timer_time;
  // Fill the sensor output message
  sensor_output_msg_.stamp = now();
  // Another item that should be moved to telemetry. This checks how long it
  // takes to fill the sensor output message
  auto start_time = now();
  // Fill the sensor output message
  for (auto& sensor_client : sensor_clients_) {
    sensor_client->FillSensorOutput(sensor_output_msg_);
  }
  // Store the time the sensor output message was filled
  auto end_time = now();
  RCLCPP_DEBUG(get_logger(), "Time to fill sensor output: %f",
               (end_time - start_time).seconds());
  // Publish the sensor output message
  sensor_output_publisher_->publish(sensor_output_msg_);
}

}  // namespace ndof_sensor_driver
}  // namespace client
}  // namespace machina

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
    machina::client::ndof_sensor_driver::MachinaNDofSensorDriverClientNode)