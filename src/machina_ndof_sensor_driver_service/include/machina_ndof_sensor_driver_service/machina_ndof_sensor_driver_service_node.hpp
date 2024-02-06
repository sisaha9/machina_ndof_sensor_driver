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

#ifndef MACHINA_NDOF_SENSOR_DRIVER_SERVICE__MACHINA_NDOF_SENSOR_DRIVER_SERVICE_NODE_HPP_
#define MACHINA_NDOF_SENSOR_DRIVER_SERVICE__MACHINA_NDOF_SENSOR_DRIVER_SERVICE_NODE_HPP_

#include <atomic>
#include <example_interfaces/msg/float64_multi_array.hpp>
#ifdef ENABLE_LOCKFREE
#include <iceoryx_hoofs/internal/concurrent/taco.hpp>
#else
#include <shared_mutex>
#endif
#include <memory>
#include <rclcpp/service.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <thread>

#include "machina_ndof_sensor_driver_msgs/srv/get_latest_sensor_data.hpp"
#include "machina_ndof_sensor_driver_service_parameters.hpp"
#include "machina_tcp_interface/machina_tcp_interface.hpp"

namespace machina {
namespace service {
namespace ndof_sensor_driver {

#ifdef ENABLE_LOCKFREE
/**
 * @brief Enum class for thread context
 *
 * Fed to the TACO during store and load operations so that it can identify
 * our context and use that to determine read/write access.
 * Specified separately for each class since each class will have it's own
 * set of contexts it needs to manage.
 */
enum class ThreadContext : uint32_t { Producer, Consumer, END_OF_LIST };
#endif

/**
 * @brief Structure to hold incoming sensor data and timestamp
 */
struct NDofSensorData {
  example_interfaces::msg::Float64MultiArray data;
  rclcpp::Time timestamp;
};

/**
 * @brief LifecycleNode to provide service to get latest sensor data
 *
 * Uses the @ref machina::tcp_interface::TcpInterface to read sensor data and
 * advertise a service to serve the latest sensor data.
 * A thread is spawned to read sensor data continuously and store it. When a
 * service request is received, the latest sensor data is served.
 * Since this is concurrent, we have defined 2 modes of operation:
 * - Lock-free using TACO from
 *   iceoryx (https://iceoryx.io/latest/advanced/iceoryx_hoofs/#concurrent)
 * - Using shared_mutex
 * You can toggle between these modes by setting -DENABLE_LOCKFREE to ON/OFF
 * ON for lock-free and OFF for shared_mutex. More experiments and data needs
 * to be collected but right now TACO enables a significant speedup but it
 * blocks access to the data often at high frequencies. Thinking of swapping to
 * a 2 element container in the future so that the producer can always write to
 * the 2nd element. If the 2nd element is being written to (can use atomic to
 * check) then the consumer can read from the 1st element. This way we can
 * avoid blocking. This is a future update.
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
 * the case of service-client relationship.
 *
 * A future update to be made here is to use TypeAdapters to reduce the
 * number of memory copies. If we have 2 nodes in the same container and
 * use type adapters, we could get the data, send it in the exact same format
 * to the other node and it can use it directly. This would be a zero copy
 *
 */
class MachinaNDofSensorDriverServiceNode
    : public rclcpp_lifecycle::LifecycleNode {
 public:
  /**
   * @brief Construct a new MachinaNDofSensorDriverServiceNode object
   *
   * The main node option I am interested in is use_intraprocess_comms. This
   * is exposed in the launch file under the package
   * machina_ndof_sensor_driver_system_tests.
   *
   * @param options NodeOptions for the node
   */
  explicit MachinaNDofSensorDriverServiceNode(
      const rclcpp::NodeOptions& options);
  /**
   * @brief Destroy the MachinaNDofSensorDriverServiceNode object
   */
  ~MachinaNDofSensorDriverServiceNode();
  /**
   * @brief Callback for the configure lifecycle transition
   *
   * Used to preallocate memory, start the TCP interface and retrieve ROS 2
   * parameters
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback for the activate lifecycle transition
   *
   * Starts the sensor data thread and the service
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback for the deactivate lifecycle transition
   *
   * Stops the sensor data thread. The service is still active but will no
   * longer fill sensor data
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback for the shutdown lifecycle transition
   *
   * Just calls the @ref on_cleanup function
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback for the cleanup lifecycle transition
   *
   * Stops the TCP connection and resets the service pointer
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  /**
   * @brief Callback for the error lifecycle transition
   *
   * Calls the @ref on_shutdown function which right now calls the cleanup
   * function
   *
   * @param previous_state The previous state of the node
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State& previous_state) override;

 private:
  /**
   * @brief Pointer to the TCP interface
   */
  std::unique_ptr<machina::tcp_interface::TcpInterface> tcp_interface_;
  /**
   * @brief Pointer to the ROS 2 parameter listener. Uses the
   * generate_parameter_library package to generate the parameters in a less
   * verbose way
   */
  std::unique_ptr<ros_parameters::ParamListener> param_listener_;
  /**
   * @brief Structure to hold the ROS 2 parameters retrieved from the parameter
   * listener
   */
  ros_parameters::Params ros_params_;

#ifdef ENABLE_LOCKFREE
  /**
   * @brief TACO to hold the latest sensor data
   *
   * If we are using TACO we don't need a class member to store the data.
   * The TACO will store the data and provide methods to read/write to it
   */
  iox::concurrent::TACO<NDofSensorData, ThreadContext> sensor_data_taco_;
#else
  /**
   * @brief Mutex to protect the latest sensor data
   *
   * If we are not using lock-free methods we need to use a mutex to protect
   * the latest sensor data. This is a shared_mutex so that multiple readers
   * can access the data at the same time
   */
  std::shared_mutex latest_sensor_data_mutex_;
  /**
   * @brief Structure to hold the latest sensor data
   *
   * If we are not using lock-free methods we need a class member to store the
   * latest sensor data to share b/w threads
   */
  NDofSensorData latest_sensor_data_;
#endif
  /**
   * @brief Service to get the latest sensor data
   */
  rclcpp::Service<machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData>::
      SharedPtr sensor_data_service_;
  /**
   * @brief Buffer to hold the sample size of the sensor data
   *
   * I allocate it once and then reuse it to avoid memory allocation overhead
   * At a future time point, we may allow the sample size to be dynamic. Though
   * it does mess with memory allocation and deallocation during the "realtime"
   * operation of the node so I am not sure if it is a good idea
   */
  uint8_t* sample_size_buffer_;
  /**
   * @brief Buffer to hold the sensor data
   *
   * I allocate it once and then reuse it to avoid memory allocation overhead
   * This doesn't need to be thread safe since the sensor data thread is the
   * only one interacting with this. The data gets passed onto
   * @ref latest_sensor_data_ if we are not using lock-free methods. Otherwise
   * it gets stored in @ref sensor_data_taco_
   */
  double* sensor_data_buffer_;
  /**
   * @brief Thread to read sensor data continuously
   *
   * This thread is started in the activate lifecycle transition and stopped
   * in the deactivate lifecycle transition
   *
   * It reads sensor data continuously and stores it in either
   * @ref latest_sensor_data_ or @ref sensor_data_taco_ depending on the
   * if we are compiling with lock-free methods or not
   */
  std::thread sensor_data_thread_;

  /**
   * @brief Atomic to check if the sensor data thread is running
   *
   * I don't use the LOCKFREE macro here since I think the atomic should always
   * be faster for a boolean. We also assert that it is always lock-free
   */
  std::atomic<bool> sensor_data_thread_running_{false};
  static_assert(std::atomic<bool>::is_always_lock_free);

  /**
   * @brief Function to read sensor data continuously
   *
   * This function is called in the sensor_data_thread_ and reads sensor data
   * continuously.
   */
  void ReadSensorDataContinuously();

  /**
   * @brief Callback for the get latest sensor data service
   *
   * This function is called when a service request is received. It serves the
   * latest sensor data
   */
  void GetLatestSensorData(
      const std::shared_ptr<
          machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData::Request>
          request,
      std::shared_ptr<
          machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData::Response>
          response);
};
}  // namespace ndof_sensor_driver
}  // namespace service
}  // namespace machina

#endif  // MACHINA_NDOF_SENSOR_DRIVER_SERVICE__MACHINA_NDOF_SENSOR_DRIVER_SERVICE_NODE_HPP_