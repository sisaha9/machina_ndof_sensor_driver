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

#ifndef MACHINA_NDOF_SENSOR_DRIVER_CLIENT__MACHINA_NDOF_SENSOR_DRIVER_CLIENT_HPP_
#define MACHINA_NDOF_SENSOR_DRIVER_CLIENT__MACHINA_NDOF_SENSOR_DRIVER_CLIENT_HPP_

#include <atomic>
#ifdef ENABLE_LOCKFREE
#include <iceoryx_hoofs/internal/concurrent/taco.hpp>
#else
#include <shared_mutex>
#endif
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>

#include "machina_ndof_sensor_driver_msgs/msg/joint_sensor_output.hpp"
#include "machina_ndof_sensor_driver_msgs/srv/get_latest_sensor_data.hpp"

namespace machina {
namespace client {
namespace ndof_sensor_driver {
#ifdef ENABLE_LOCKFREE
/**
 * @brief Enum to specify the context of the thread
 *
 * Fed to the TACO during store and load operations so that it can identify
 * our context and use that to determine read/write access.
 * Specified separately for each class since each class will have it's own
 * set of contexts it needs to manage.
 */
enum class ThreadContext : uint32_t { Producer, Consumer, END_OF_LIST };
#endif

/**
 * @brief Class to handle the client side of the Machina NDof Sensor Driver
 *
 * This class provides an interface to the Machina NDof Sensor Driver to
 * request sensor data and store it for further processing.
 *
 * The @ref MachinaNDofSensorDriverClientNode class spawns multiple instances
 * of this class to handle multiple sensors.
 *
 * The class uses a client to request one sensor's data from the driver and
 * stores it. The data is then sent to the
 * @ref MachinaNDofSensorDriverClientNode via the @ref FillSensorOutput method.
 *
 * A thread is spawned to continuously request data from the driver at a
 * specified interval. Since this is concurrent with the main thread, we
 * have defined 2 modes of operation:
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
 * A future update to be made here is to use TypeAdapters to reduce the
 * number of memory copies. If we have 2 nodes in the same container and
 * use type adapters, we could get the data, send it in the exact same format
 * to the other node and it can use it directly. This would be a zero copy
 *
 * Additionally, unlike @ref MachinaNDofSensorDriverServiceNode, this class
 * and @ref MachinaNDofSensorDriverClientNode have multiple callbacks sent to
 * the ROS 2 executor. Hence, a simple StaticSingleThreadedExecutor (defined
 * in the CMake of the @ref MachinaNDofSensorDriverServiceNode) won't work
 * here if we want to hit high frequencies. We use a MultiThreadedExecutor
 * (defined in the CMake) and set a new callback group here so that the
 * executor runs at least the callbacks defined here in parallel with other
 * callbacks on the executor.
 */
class MachinaNDofSensorDriverClient {
 public:
  /**
   * @brief Construct a new MachinaNDofSensorDriverClient object
   *
   * @param node Pointer to the lifecycle node for the
   * @ref MachinaNDofSensorDriverClientNode
   * @param idx Index of the sensor. Used to identify which index to fill
   * in the @ref JointSensorOutput message
   * @param collection_interval Interval at which to request data from the
   * driver
   * @param service_timeout_interval Timeout interval for the service during
   * initial connection
   * @param max_service_connection_attempts Maximum number of attempts to
   * connect to the service before throwing an error during initial connection
   */
  explicit MachinaNDofSensorDriverClient(
      rclcpp_lifecycle::LifecycleNode* node, uint64_t idx,
      uint64_t collection_interval, uint64_t service_timeout_interval,
      uint64_t max_service_connection_attempts);
  /**
   * @brief Destroy the MachinaNDofSensorDriverClient object
   */
  ~MachinaNDofSensorDriverClient() = default;
  /**
   * @brief Fill the sensor output message with the latest sensor data
   *
   * This method is used by the @ref MachinaNDofSensorDriverClientNode to
   * fill the sensor output message with the latest sensor data. The message
   * is then published to the ROS 2 network.
   *
   * @param msg Reference to the sensor output message to be filled
   */
  void FillSensorOutput(
      machina_ndof_sensor_driver_msgs::msg::JointSensorOutput& msg);
  /**
   * @brief Start the thread to continuously request sensor data
   *
   * This method starts the thread to continuously request sensor data from
   * the driver at the specified interval. The data is then stored for
   * further processing.
   */
  void StartReadingSensor();

  /**
   * @brief Stop the thread to continuously request sensor data
   *
   * This method stops the thread to continuously request sensor data from
   * the driver at the specified interval.
   */
  void StopReadingSensor();

 private:
  /**
   * @brief Index of the sensor. Used to identify which index to fill in the
   * @ref JointSensorOutput message
   */
  uint64_t index_;
  /**
   * @brief Interval at which to request data from the driver
   */
  uint64_t collection_interval_;
  /**
   * @brief Callback group for the client. Used to run the client callbacks
   * in parallel with other callbacks on the executor
   */
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  /**
   * @brief Client to request sensor data from the driver
   */
  rclcpp::Client<machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData>::
      SharedPtr sensor_data_client_;
  /**
   * @brief Empty request to send to the driver to request sensor data
   */
  machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData::Request::SharedPtr
      empty_request_;
#ifdef ENABLE_LOCKFREE
  /**
   * @brief TACO to store the sensor data
   *
   * If we are using lock-free, we use a TACO to store the sensor data and
   * don't need to use a shared_mutex to protect the data nor a class member
   * variable to store the data. The TACO is used to store the sensor data
   */
  iox::concurrent::TACO<machina_ndof_sensor_driver_msgs::msg::SensorOutput,
                        ThreadContext>
      sensor_data_taco_;
#else
  /**
   * @brief Shared Mutex to protect the sensor data
   *
   * If we are not using lock-free, we use a shared_mutex to protect the
  // @ref latest_sensor_data_ variable
  */
  std::shared_mutex latest_sensor_data_mutex_;
  /**
   * @brief Latest sensor data
   *
   * If we are not using lock-free, we need a class member variable to store
   * the latest sensor data and exchange b/w threads
   */
  machina_ndof_sensor_driver_msgs::msg::SensorOutput latest_sensor_data_;
#endif
  /**
   * @brief Thread to continuously request sensor data
   */
  std::thread sensor_data_thread_;
  /**
   * @brief Atomic to check if the sensor data thread is running
   *
   * I don't use the LOCKFREE macro here since I think the atomic should always
   * be faster for a boolean. We also assert that it is always lock-free
   */
  std::atomic<bool> sensor_data_thread_running_{false};
  // Assert that the atomic is always lock-free for a boolean
  static_assert(std::atomic<bool>::is_always_lock_free);

  /**
   * @brief Read sensor data continuously
   *
   * This method is called by the sensor data thread to continuously request
   * sensor data from the driver at the specified interval
   */
  void ReadSensorDataContinuously();
};
}  // namespace ndof_sensor_driver
}  // namespace client
}  // namespace machina

#endif  // MACHINA_NDOF_SENSOR_DRIVER_CLIENT__MACHINA_NDOF_SENSOR_DRIVER_CLIENT_HPP_