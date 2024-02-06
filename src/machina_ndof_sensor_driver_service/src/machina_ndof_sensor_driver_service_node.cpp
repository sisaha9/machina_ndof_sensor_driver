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

#include "machina_ndof_sensor_driver_service/machina_ndof_sensor_driver_service_node.hpp"

#include <algorithm>
#include <chrono>
#include <example_interfaces/msg/multi_array_dimension.hpp>
#include <example_interfaces/msg/multi_array_layout.hpp>

namespace machina {
namespace service {
namespace ndof_sensor_driver {

/******************************************************************************/
MachinaNDofSensorDriverServiceNode::MachinaNDofSensorDriverServiceNode(
    const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("machina_ndof_sensor_driver_service_node",
                                      options)
#ifdef ENABLE_LOCKFREE
      ,
      sensor_data_taco_(iox::concurrent::TACOMode::DenyDataFromSameContext)
#endif
{
  // Initialize the TCP interface and the parameter listener
  tcp_interface_ = std::make_unique<machina::tcp_interface::TcpInterface>();
  param_listener_ = std::make_unique<ros_parameters::ParamListener>(
      this->get_node_parameters_interface());
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Service Node Constructed");
}

/******************************************************************************/
MachinaNDofSensorDriverServiceNode::~MachinaNDofSensorDriverServiceNode() {
  // Clean up the TCP interface and the parameter listener
  // Also delete the sample size buffer which is the only new object created
  delete[] sample_size_buffer_;
  tcp_interface_.reset();
  param_listener_.reset();
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverServiceNode::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Retrieve the parameters. Right now all parameters are frozen on startup
  // so there is no need to call this function again
  ros_params_ = param_listener_->get_params();
  // Create the sample size buffer. It will store 8 bytes to send as a sample
  // size
  sample_size_buffer_ = new uint8_t[sizeof(uint64_t)];
  // Convert the requested sample size to a byte array. Since the sample size
  // is constant for the lifetime of the node, we can do this once
  uint64_t req_sample_size =
      static_cast<uint64_t>(ros_params_.requested_sample_size);
  // Copy the bytes to the allocated buffer
  std::memcpy(sample_size_buffer_, &req_sample_size, sizeof(uint64_t));
  // Attempt to connect to the sensor. If the connection fails, retry
  // @ref ros_params_.max_connection_attempts times
  uint64_t num_attempts = 0;
  while (!tcp_interface_->StartConnection(ros_params_.sensor.ip_addr,
                                          ros_params_.sensor.port) &&
         num_attempts <=
             static_cast<uint64_t>(ros_params_.max_connection_attempts)) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to sensor. Retrying...");
    // Sleep for the connection retry interval
    std::this_thread::sleep_for(
        std::chrono::milliseconds(ros_params_.connection_retry_interval));
    num_attempts++;
  }
  // If the number of attempts exceeds the maximum allowed, log a fatal error
  // and return failure
  if (num_attempts >
      static_cast<uint64_t>(ros_params_.max_connection_attempts)) {
    RCLCPP_FATAL(get_logger(), "Failed to connect to sensor. Exiting...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Service Node Configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverServiceNode::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Advertising the service to respond to requests for the latest sensor data
  sensor_data_service_ = this->create_service<
      machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData>(
      "get_latest_sensor_data",
      std::bind(&MachinaNDofSensorDriverServiceNode::GetLatestSensorData, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS());
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Service Node Activated");
  // Start the thread to read sensor data continuously
  sensor_data_thread_running_.store(true);
  sensor_data_thread_ = std::thread(
      &MachinaNDofSensorDriverServiceNode::ReadSensorDataContinuously, this);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverServiceNode::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Stop the thread to read sensor data continuously
  sensor_data_thread_running_.store(false);
  sensor_data_thread_.join();
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Service Node Deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverServiceNode::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Call the cleanup function to close the connection and reset the service
  on_cleanup(previous_state);
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Service Node Shut Down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverServiceNode::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Close the TCP connection and reset the service
  tcp_interface_->CloseConnection();
  sensor_data_service_.reset();
  RCLCPP_INFO(get_logger(),
              "Machina NDof Sensor Driver Service Node Cleaned Up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MachinaNDofSensorDriverServiceNode::on_error(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  // Call the shutdown function to clean up and log an error message
  on_shutdown(previous_state);
  RCLCPP_INFO(get_logger(), "Machina NDof Sensor Driver Service Node Errored");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

/******************************************************************************/
void MachinaNDofSensorDriverServiceNode::ReadSensorDataContinuously() {
#ifndef ENABLE_LOCKFREE
  // If we are not using lockfree data structures, we need to lock the mutex
  // before allocating the latest sensor data since the ROS 2 service callback
  // will be reading from it
  latest_sensor_data_mutex_.lock();
#else
  // If we are using lockfree data structures, we need to allocate the latest
  // sensor data and store it in the TACO before starting to read sensor data
  NDofSensorData latest_sensor_data_;
#endif
  // Set to uninitialized as a way to keep track of if we have received any
  // sensor data
  latest_sensor_data_.timestamp = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
  // Allocate the data vector to store the sensor data. It is 2D with the first
  // dimension being the degrees of freedom and the second dimension being the
  // requested sample size
  latest_sensor_data_.data.data.reserve(ros_params_.requested_sample_size *
                                        ros_params_.degrees_of_freedom);
  // Set the layout of the data
  example_interfaces::msg::MultiArrayLayout layout;
  example_interfaces::msg::MultiArrayDimension dim;
  dim.size = ros_params_.degrees_of_freedom;
  dim.stride = ros_params_.requested_sample_size;
  dim.label = "degrees_of_freedom";
  latest_sensor_data_.data.layout.dim.push_back(dim);
  dim.size = ros_params_.requested_sample_size;
  dim.stride = 1;
  dim.label = "requested_sample_size";
  latest_sensor_data_.data.layout.dim.push_back(dim);
  latest_sensor_data_.data.layout.data_offset = 0;
#ifndef ENABLE_LOCKFREE
  // If we are not using lockfree data structures, we need to unlock the mutex
  // before starting to read sensor data. The ROS 2 service can read the data
  // now if it was trying to. But it will see the uninitialized timestamp and
  // know that the data is not yet available
  latest_sensor_data_mutex_.unlock();
#else
  // If we are using lockfree data structures, we need to store the latest
  // sensor data in the TACO before starting to read sensor data. The ROS 2
  // service can read the data now if it was trying to. But it will see the
  // uninitialized timestamp and know that the data is not yet available.
  sensor_data_taco_.store(latest_sensor_data_, ThreadContext::Producer);
#endif
  // Start reading sensor data continuously. We are using the boolean flag to
  // control the loop. If the flag is set to false, we will exit the loop and
  // stop reading sensor data enabling a clean shutdown
  while (sensor_data_thread_running_.load()) {
    // Debug purposes to see how long it takes to read sensor data
    auto start_time = now();
    // Send the sample size to the sensor
    bool send_res =
        tcp_interface_->SendData(sample_size_buffer_, sizeof(uint64_t));
    // If the number of bytes sent is not equal to the size of the sample size
    // buffer, log an error. Undefined what happens with the sensor now. We
    // do check the number of bytes received from the sensor later on so we can
    // validate if something went wrong
    if (!send_res) {
      RCLCPP_ERROR(get_logger(), "Sent wrong number of bytes to sensor");
    }
#ifndef ENABLE_LOCKFREE
    // If we are not using lockfree data structures, we need to lock the mutex
    // before allocating the latest sensor data since the ROS 2 service
    // callback will be reading from it
    latest_sensor_data_mutex_.lock();
#endif
    // Allocate the data buffer to store the sensor data
    uint8_t* data_buffer =
        reinterpret_cast<uint8_t*>(latest_sensor_data_.data.data.data());
    uint64_t data_len = ros_params_.requested_sample_size *
                        ros_params_.degrees_of_freedom * sizeof(double);
    // Receive the sensor data
    uint64_t received_len = tcp_interface_->ReceiveData(data_buffer, data_len);
    // If the number of bytes received is not equal to the size of the data
    // buffer, log an error and continue. Undefined what we have received
    if (received_len != ros_params_.requested_sample_size *
                            ros_params_.degrees_of_freedom * sizeof(double)) {
      RCLCPP_ERROR(get_logger(), "Received wrong number of bytes from sensor");
      RCLCPP_ERROR(get_logger(), "Expected %ld and received %ld",
                   ros_params_.requested_sample_size *
                       ros_params_.degrees_of_freedom * sizeof(double),
                   received_len);
      continue;
    }
    // If we have reached here things look good so far
    // Set the timestamp to the current time. Ideally, the sensor should be
    // PTP synchronized with the ROS 2 system and we should get the hardware
    // timestamp from when the sensor hit the network
    // But for now, we will use the time we received
    latest_sensor_data_.timestamp = now();
    // Set the data vector to the received data buffer
    latest_sensor_data_.data.data = std::vector<double>(
        reinterpret_cast<double*>(data_buffer),
        reinterpret_cast<double*>(data_buffer) +
            ros_params_.requested_sample_size * ros_params_.degrees_of_freedom);
#ifdef ENABLE_LOCKFREE
    // If we are using lockfree data structures, we need to store the latest
    // sensor data in the TACO
    sensor_data_taco_.store(latest_sensor_data_, ThreadContext::Producer);
#else
    // If we are not using lockfree data structures, we need to unlock the mutex
    // From here we will no longer be modifying the latest sensor data so we can
    // unlock the mutex and use a shared lock to allow multiple readers
    latest_sensor_data_mutex_.unlock();
    latest_sensor_data_mutex_.lock_shared();
#endif
    // Check if the data received is corrupted. We expect the last value to be
    // the sum of all the other values. NOTE: This is not the case in the
    // sensor simulation given. I modified it to act as a CRC of sorts
    double sum_of_all_vals_except_end =
        std::accumulate(latest_sensor_data_.data.data.begin(),
                        latest_sensor_data_.data.data.end() - 1, 0.0);
    if ((sum_of_all_vals_except_end -
         latest_sensor_data_.data.data[ros_params_.requested_sample_size *
                                           ros_params_.degrees_of_freedom -
                                       1]) > 1e-6) {
      RCLCPP_ERROR(
          get_logger(), "Data received but corrupted %f %f",
          sum_of_all_vals_except_end,
          latest_sensor_data_.data.data[ros_params_.requested_sample_size *
                                            ros_params_.degrees_of_freedom -
                                        1]);
    }
#ifndef ENABLE_LOCKFREE
    // If we are not using lockfree data structures, we are done reading and
    // validating the sensor data. We can unlock the shared lock now
    latest_sensor_data_mutex_.unlock_shared();
#endif
    // Debug purposes to see how long it takes to read sensor data. Ideally
    // should go on a telemetry publisher
    auto end_time = now();
    int64_t dur =
        std::max(static_cast<int64_t>(0),
                 static_cast<int64_t>((end_time - start_time).nanoseconds()));
    RCLCPP_DEBUG(get_logger(), "Took %f seconds to read sensor data",
                 dur / 1e9);
    // Sleep for the polling interval. We will sleep for the remaining time
    // after we have read the sensor data
    auto sleep_time = std::max(static_cast<int64_t>(0),
                               ros_params_.sensor_polling_interval - dur);
    std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
  }
}

/******************************************************************************/
void MachinaNDofSensorDriverServiceNode::GetLatestSensorData(
    const std::shared_ptr<
        machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData::Request>
        request,
    std::shared_ptr<
        machina_ndof_sensor_driver_msgs::srv::GetLatestSensorData::Response>
        response) {
  (void)request;
#ifdef ENABLE_LOCKFREE
  // If we are using lockfree data structures, we need to get the latest sensor
  // data from the TACO
  NDofSensorData latest_sensor_data_;
  auto ret_val = sensor_data_taco_.take(ThreadContext::Consumer);
  if (ret_val.has_value()) {
    latest_sensor_data_ = *ret_val;
  } else {
    response->sensor_data_filled = false;
    RCLCPP_WARN(get_logger(), "Failed to get sensor data from TACO");
    return;
  }
#else
  // If we are not using lockfree data structures, we need to share lock before
  // reading the latest sensor data
  latest_sensor_data_mutex_.lock_shared();
#endif
  // Check if the sensor data has been received. If not, log an error and
  // set the response to false
  if (latest_sensor_data_.timestamp.get_clock_type() ==
      RCL_CLOCK_UNINITIALIZED) {
    response->sensor_data_filled = false;
    RCLCPP_ERROR(get_logger(), "Sensor data not yet received");
  } else if (!sensor_data_thread_running_.load()) {
    // If the sensor data has been received but the thread is not running, log
    // an error and set the response to false
    response->sensor_data_filled = false;
    RCLCPP_ERROR(
        get_logger(),
        "Sensor data not being collected. Likely in deactivated state");
  } else {
    // If the sensor data has been received, set the response to true and
    // fill the sensor data
    response->sensor_data_filled = true;
    response->sensor.data = latest_sensor_data_.data;
    response->sensor.header.stamp = latest_sensor_data_.timestamp;
    response->sensor.header.frame_id = ros_params_.frame_id;
  }
#ifndef ENABLE_LOCKFREE
  // If we are not using lockfree data structures, we need to unlock the shared
  // lock now
  latest_sensor_data_mutex_.unlock_shared();
#endif
}

}  // namespace ndof_sensor_driver
}  // namespace service
}  // namespace machina

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
    machina::service::ndof_sensor_driver::MachinaNDofSensorDriverServiceNode)