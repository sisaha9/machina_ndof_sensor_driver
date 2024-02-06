// Copyright 2023 Siddharth Saha.

// Licensed under the GNU Affero General Public License, v3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     https://www.gnu.org/licenses/agpl-3.0.html

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// SPDX-License-Identifier: AGPL-3.0-only

#include "machina_ndof_sensor_driver_client/machina_ndof_sensor_driver_client.hpp"

#include <algorithm>
#include <chrono>
#include <example_interfaces/msg/multi_array_dimension.hpp>
#include <example_interfaces/msg/multi_array_layout.hpp>

namespace machina {
namespace client {
namespace ndof_sensor_driver {

/******************************************************************************/
MachinaNDofSensorDriverClient::MachinaNDofSensorDriverClient(
    rclcpp_lifecycle::LifecycleNode* node, uint64_t idx,
    uint64_t collection_interval, uint64_t service_timeout_interval,
    uint64_t max_service_connection_attempts)
    : index_(idx),
      collection_interval_(collection_interval),
      client_callback_group_(node->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive)),
      sensor_data_client_(
          node->create_client<
              machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData>(
              "get_latest_sensor_data_" + std::to_string(idx),
              rclcpp::ServicesQoS(), client_callback_group_)),
      empty_request_(std::make_shared<machina_ndof_sensor_driver_msgs::srv::
                                          GetLatestSensorData::Request>())
#ifdef ENABLE_LOCKFREE
      ,
      sensor_data_taco_(iox::concurrent::TACOMode::DenyDataFromSameContext)
#endif
{
  // Wait for service to be available
  uint64_t attempts = 0;
  while (!sensor_data_client_->wait_for_service(
      std::chrono::nanoseconds(service_timeout_interval))) {
    std::cout << "Waiting for service at index " << std::to_string(idx)
              << std::endl;
    attempts++;
    // If service is not available after max attempts, throw an error
    if (attempts >= max_service_connection_attempts) {
      throw std::runtime_error("Service not available at index " +
                               std::to_string(idx));
      return;
    }
  }
}

/******************************************************************************/
void MachinaNDofSensorDriverClient::FillSensorOutput(
    machina_ndof_sensor_driver_msgs::msg::JointSensorOutput& msg) {
  // Check if the index is within the range of the vector
  if (index_ >= msg.sensors.size()) {
    std::cerr << "Joint Sensor Output Vector not preallocated correctly"
              << std::endl;
    return;
  }
  // Check if the sensor data thread is running
  if (!sensor_data_thread_running_.load()) {
    std::cerr << "Sensor data not being collected. Likely in deactivated state"
              << std::endl;
    return;
  }
#ifndef ENABLE_LOCKFREE
  // If we are not using lockfree, we need to share lock the mutex to access the
  // latest sensor data. If we haven't received any sensor data yet, this
  // will just be empty data
  latest_sensor_data_mutex_.lock_shared();
  msg.sensors.at(index_) = latest_sensor_data_;
  latest_sensor_data_mutex_.unlock_shared();
#else
  // If we are using lockfree, we can just take the sensor data from the TACO
  // TACO can check if it is safe to take the data and if any data is available
  auto ret_val = sensor_data_taco_.take(ThreadContext::Consumer);
  if (ret_val.has_value()) {
    msg.sensors.at(index_) = *ret_val;
  } else {
    std::cerr << "Failed to get sensor data from TACO: "
              << std::to_string(index_) << std::endl;
  }
#endif
}

/******************************************************************************/
void MachinaNDofSensorDriverClient::StartReadingSensor() {
  // Start the sensor data thread
  sensor_data_thread_running_.store(true);
  sensor_data_thread_ = std::thread(
      &MachinaNDofSensorDriverClient::ReadSensorDataContinuously, this);
}

/******************************************************************************/
void MachinaNDofSensorDriverClient::StopReadingSensor() {
  // Stop the sensor data thread
  sensor_data_thread_running_.store(false);
  sensor_data_thread_.join();
}

/******************************************************************************/
void MachinaNDofSensorDriverClient::ReadSensorDataContinuously() {
  // Continuously request sensor data from the driver
  bool can_send_request = true;
  // Loop until the thread is running. We use a boolean atomic to check if the
  // thread should still be running. If we want to stop this thread we can
  // set the boolean to false and it will enable a clean shutdown of this thread
  while (sensor_data_thread_running_.load()) {
    // If we can send a request, send a request to the driver
    if (can_send_request) {
      // Start the timer to measure the time it takes to get the sensor data
      auto start_time = std::chrono::high_resolution_clock::now();
      // Set the boolean to false so we don't send another request until we get
      // the response
      can_send_request = false;
      // Send the request to the driver
      // Async handler to not block the thread while waiting for the response
      auto future_result = sensor_data_client_->async_send_request(
          empty_request_,
          [this, &start_time, &can_send_request](
              rclcpp::Client<machina_ndof_sensor_driver_msgs::srv::
                                 GetLatestSensorData>::SharedFuture future) {
            // Get the sensor data from the future
            auto sensor_data = future.get();
            // If the sensor data is filled, store it in the class member
            // variable
            if (sensor_data->sensor_data_filled) {
#ifndef ENABLE_LOCKFREE
              // If we are not using lockfree, we need to lock the mutex to
              // store the sensor data
              latest_sensor_data_mutex_.lock();
              latest_sensor_data_ = sensor_data->sensor;
              latest_sensor_data_mutex_.unlock();
#else
              // If we are using lockfree, we can just store the sensor data in
              // the TACO
              sensor_data_taco_.store(sensor_data->sensor,
                                      ThreadContext::Producer);
#endif
            } else {
              // If the sensor data is not filled, print an error. This happens
              // often when the service is using TACO and the data is not
              // safe to take
              std::cerr << "Sensor data not filled in index "
                        << std::to_string(index_) << std::endl;
            }
            // End the timer to measure the time it takes to get the sensor data
            auto end_time = std::chrono::high_resolution_clock::now();
            int64_t dur = std::max(
                static_cast<int64_t>(0),
                static_cast<int64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        end_time - start_time)
                        .count()));
            // Sleep for the remaining time to get the sensor data at the
            // specified interval
            auto sleep_time =
                std::max(static_cast<int64_t>(0),
                         static_cast<int64_t>(collection_interval_ - dur));
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
            // Set the boolean to true so we can send another request
            can_send_request = true;
          });
    }
  }
}
}  // namespace ndof_sensor_driver
}  // namespace client
}  // namespace machina