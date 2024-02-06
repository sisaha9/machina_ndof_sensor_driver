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

#ifndef MACHINA_NDOF_SENSOR_DRIVER_CLIENT__MACHINA_NDOF_SENSOR_DRIVER_CLIENT_NODE_HPP_
#define MACHINA_NDOF_SENSOR_DRIVER_CLIENT__MACHINA_NDOF_SENSOR_DRIVER_CLIENT_NODE_HPP_

#include <memory>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "machina_ndof_sensor_driver_client/machina_ndof_sensor_driver_client.hpp"
#include "machina_ndof_sensor_driver_client_parameters.hpp"
#include "machina_ndof_sensor_driver_msgs/msg/joint_sensor_output.hpp"
#include "machina_ndof_sensor_driver_msgs/srv/get_latest_sensor_data.hpp"

namespace machina {
namespace client {
namespace ndof_sensor_driver {

/**
 * @brief Class to handle the client side of the Machina NDof Sensor Driver
 *
 * This class provides an interface to the Machina NDof Sensor Driver to
 * request sensor data and publish it for further processing.
 *
 * This class spawns multiple instances of @ref MachinaNDofSensorDriverClient
 * handle multiple sensors. It periodically polls each client for the latest
 * sensor data and publishes it.
 *
 * The class specifies it's own callback group for the timer so that it can
 * run in parallel to the client-service callbacks in each instance of the
 * @ref MachinaNDofSensorDriverClient class.
 *
 * The node also uses the Lifecycle Design Pattern
 * (https://design.ros2.org/articles/node_lifecycle.html)
 * That means the node won't start just by the typical ros2 run routine. You
 * need to call the lifecycle transitions configure and activate to enter
 * nominal operation. Refer to the documentation for more details.
 * https://github.com/ros2/demos/blob/iron/lifecycle/README.rst
 *
 * The node also uses the Composition Design Pattern
 * (https://docs.ros.org/en/iron/Concepts/Intermediate/About-Composition.html)
 * Ideally, this would mean there is a zero copy transfer of data between nodes
 * in the same component container. However, I am not sure if that applies in
 * the case of service-client relationship. But for anyone subscribing to the
 * output of this node, it should be a zero copy transfer.
 */
class MachinaNDofSensorDriverClientNode
    : public rclcpp_lifecycle::LifecycleNode {
 public:
  /**
   * @brief Construct a new MachinaNDofSensorDriverClientNode object
   *
   * The main node option I am interested in is use_intraprocess_comms. This
   * is exposed in the launch file under the package
   * machina_ndof_sensor_driver_system_tests.
   *
   * @param options NodeOptions object to specify the parameters of the node
   */
  explicit MachinaNDofSensorDriverClientNode(
      const rclcpp::NodeOptions& options);
  /**
   * @brief Destroy the MachinaNDofSensorDriverClientNode object
   */
  ~MachinaNDofSensorDriverClientNode() = default;
  /**
   * @brief Callback to configure the node
   *
   * Retrieves ROS 2 parameters from the listener, preallocates memory,
   * initializes each sensor client and creates the publishers for the sensors
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback to activate the node
   *
   * Starts the read sensor data in all clients and a timer. Also activates
   * the publisher
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback to deactivate the node
   *
   * Stops the read sensor data in all clients and the timer. Also deactivates
   * the publisher
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback to shutdown the node
   *
   * Just call's the @ref on_cleanup method
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback to cleanup the node
   *
   * Resets all the clients, timers and publishers
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback when the node is in the error state
   *
   * Just call's the @ref on_shutdown method
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State& previous_state) override;

 private:
  /**
   * @brief Pointer to the ROS 2 parameter listener. Uses the
   * generate_parameter_library package to generate the parameters in a less
   * verbose way
   */
  std::unique_ptr<ros_parameters::ParamListener> param_listener_;
  /**
   * @brief Structure to hold the ROS 2 parameters from the listener
   */
  ros_parameters::Params ros_params_;
  /**
   * @brief Vector of unique pointers to the sensor clients
   */
  std::vector<std::unique_ptr<MachinaNDofSensorDriverClient>> sensor_clients_;
  /**
   * @brief Message to hold the sensor output to publish
   */
  machina_ndof_sensor_driver_msgs::msg::JointSensorOutput sensor_output_msg_;
  /**
   * @brief Publisher for the sensor output
   *
   * Lifecycle publishers need to be activated and deactivated
   */
  rclcpp_lifecycle::LifecyclePublisher<
      machina_ndof_sensor_driver_msgs::msg::JointSensorOutput>::SharedPtr
      sensor_output_publisher_;
  /**
   * @brief Timer to periodically publish the sensor output
   */
  rclcpp::TimerBase::SharedPtr sensor_output_timer_;
  /**
   * @brief Time at which the timer last triggered. USed to debug multi threaded
   * execution in ROS 2
   */
  rclcpp::Time timer_time_;
  /**
   * @brief Callback group for the timer
   *
   * Used to run the timer in parallel to the client-service callbacks
   */
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  /**
   * @brief Publish the latest sensor data
   *
   * This method is called by the timer to publish the latest sensor data
   * collected by the clients
   */
  void PublishLatestSensorData();
};
}  // namespace ndof_sensor_driver
}  // namespace client
}  // namespace machina

#endif  // MACHINA_NDOF_SENSOR_DRIVER_CLIENT__MACHINA_NDOF_SENSOR_DRIVER_CLIENT_NODE_HPP_