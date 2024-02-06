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

#include "machina_tcp_interface/machina_tcp_interface.hpp"

#include <sys/select.h>
#include <unistd.h>

#include <iostream>

namespace machina {
namespace tcp_interface {
/******************************************************************************/
TcpInterface::TcpInterface() {
  // Initialize socket file descriptor to -1
  // -1 is used to indicate that the socket is not connected
  socket_fd_ = -1;
}

/******************************************************************************/
TcpInterface::~TcpInterface() {
  // Close the connection if it is still open
  if (IsSocketConnected()) {
    CloseConnection();
  }
}

/******************************************************************************/
bool TcpInterface::StartConnection(const std::string &sensor_ip,
                                   uint16_t port) {
  // Create a socket using IPv4 and TCP
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  // Check if the socket was created successfully
  if (socket_fd_ < 0) {
    std::cout << "Error: Could not create socket" << std::endl;
    return false;
  }
  // Set the server address
  struct sockaddr_in server_addr;
  // Set the address family to IPv4
  server_addr.sin_family = AF_INET;
  // Set the port number
  server_addr.sin_port = htons(port);
  // Set the IP address
  server_addr.sin_addr.s_addr = inet_addr(sensor_ip.c_str());
  // Convert the IP address to binary form
  if (inet_pton(AF_INET, sensor_ip.c_str(), &server_addr.sin_addr) <= 0) {
    std::cout << "Error: Invalid address/ Address not supported" << std::endl;
    return false;
  }
  // Connect to the server
  if (connect(socket_fd_, (struct sockaddr *)&server_addr,
              sizeof(server_addr)) < 0) {
    std::cout << "Error: Connection Failed" << std::endl;
    return false;
  }
  return true;
}

/******************************************************************************/
bool TcpInterface::SendData(uint8_t *data, uint64_t len) {
  // The send function returns the number of bytes sent
  // We classify a successful send as one where the number of bytes sent is
  // equal to the length of the data
  return static_cast<uint64_t>(send(socket_fd_, data, len, 0)) == len;
}

/******************************************************************************/
bool TcpInterface::SendData(uint8_t *data, uint64_t len, uint64_t timeout) {
  // Set the file descriptor set to write
  fd_set write_fds;
  FD_ZERO(&write_fds);
  FD_SET(socket_fd_, &write_fds);
  // Set the timeout
  struct timeval tv;
  tv.tv_sec = timeout * 1e-6;
  tv.tv_usec = timeout % static_cast<uint64_t>(1e6);
  // Check if the socket is ready to write. If it is, send the data
  int send_res = select(socket_fd_ + 1, NULL, &write_fds, NULL, &tv);
  if (send_res == -1) {
    std::cout << "Error during sned" << std::endl;
    return false;
  } else if (send_res == 0) {
    std::cout << "Error: Timeout occurred while sending data" << std::endl;
    return false;
  } else {
    return SendData(data, len);
  }
}

/******************************************************************************/
uint64_t TcpInterface::ReceiveData(uint8_t *data, uint64_t len) {
  // The receive function returns the number of bytes received
  uint64_t len_received = recv(socket_fd_, data, len, 0);
  return len_received;
}

/******************************************************************************/
uint64_t TcpInterface::ReceiveData(uint8_t *data, uint64_t len,
                                   uint64_t timeout) {
  // Set the file descriptor set to read
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(socket_fd_, &read_fds);
  // Set the timeout
  struct timeval tv;
  tv.tv_sec = timeout * 1e-6;
  tv.tv_usec = timeout % static_cast<uint64_t>(1e6);
  // Check if the socket is ready to read. If it is, receive the data
  int recv_res = select(socket_fd_ + 1, &read_fds, NULL, NULL, &tv);
  if (recv_res == -1) {
    std::cout << "Error during receive" << std::endl;
    return 0;
  } else if (recv_res == 0) {
    std::cout << "Error: Timeout occurred while receiving data" << std::endl;
    return 0;
  } else {
    return ReceiveData(data, len);
  }
}

/******************************************************************************/
bool TcpInterface::IsSocketConnected() {
  // Check if the socket file descriptor is valid
  if (socket_fd_ == -1) {
    return false;
  }
  int error = 0;
  socklen_t len = sizeof(error);
  // Get the socket options
  int retval = getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, &len);
  // Check if the socket options were retrieved successfully
  if (retval != 0) {
    std::cout << "Error: Could not get socket options" << std::endl;
    return false;
  }
  return error == 0;
}

/******************************************************************************/
bool TcpInterface::CloseConnection() {
  // Close the socket file descriptor if it is valid
  if (socket_fd_ != -1) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  return true;
}
}  // namespace tcp_interface
}  // namespace machina