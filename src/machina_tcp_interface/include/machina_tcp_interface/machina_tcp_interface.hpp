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

#ifndef MACHINA_TCP_INTERFACE__TCP_INTERFACE_HPP_
#define MACHINA_TCP_INTERFACE__TCP_INTERFACE_HPP_

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <cstdint>
#include <string>

namespace machina {
namespace tcp_interface {

/**
 * @brief Class to handle TCP communication with a sensor
 *
 * This class provides an interface to establish a TCP connection with a sensor
 * and send and receive data from it.
 */
class TcpInterface {
 public:
  /**
   * @brief Construct a new Tcp Interface object
   */
  TcpInterface();
  /**
   * @brief Destroy the Tcp Interface object
   */
  ~TcpInterface();

  /**
   * @brief Start a TCP connection with the sensor
   *
   * @param sensor_ip IP address of the sensor
   * @param port Port number of the sensor
   */
  bool StartConnection(const std::string& sensor_ip, uint16_t port);
  /**
   * @brief Send data to the sensor
   *
   * Doesn't do any validation and assumes the data is valid and len is correct
   * @param data Pointer to the data to be sent
   * @param len Length of the data to be sent
   * @return true If the data was sent successfully
   */
  bool SendData(uint8_t* data, uint64_t len);
  /**
   * @brief Send data to the sensor with a timeout
   *
   * Same assumptions as SendData
   *
   * @param data Pointer to the data to be sent
   * @param len Length of the data to be sent
   * @param timeout Timeout in microseconds
   * @return true If the data was sent successfully
   */
  bool SendData(uint8_t* data, uint64_t len, uint64_t timeout);
  /**
   * @brief Receive data from the sensor
   *
   * Assumes the buffer is valid and len is correct
   *
   * @param data Pointer to the buffer to store the received data
   * @param len Length of the data to be received
   * @return uint64_t Number of bytes received
   */
  uint64_t ReceiveData(uint8_t* data, uint64_t len);
  /**
   * @brief Receive data from the sensor with a timeout
   *
   * Same assumptions as ReceiveData
   *
   * @param data Pointer to the buffer to store the received data
   * @param len Length of the data to be received
   * @param timeout Timeout in microseconds
   * @return uint64_t Number of bytes received
   */
  uint64_t ReceiveData(uint8_t* data, uint64_t len, uint64_t timeout);
  /**
   * @brief Check if the socket is connected
   *
   * @return true If the socket is connected
   */
  bool IsSocketConnected();
  /**
   * @brief Close the TCP connection
   *
   * @return true If the connection was closed successfully
   */
  bool CloseConnection();

 private:
  /**
   * @brief File descriptor for the socket
   */
  int32_t socket_fd_;
};
}  // namespace tcp_interface
}  // namespace machina

#endif  // MACHINA_TCP_INTERFACE__TCP_INTERFACE_HPP_