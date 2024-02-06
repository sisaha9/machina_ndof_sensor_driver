# Copyright 2023 Siddharth Saha.

# Licensed under the GNU Affero General Public License, v3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     https://www.gnu.org/licenses/agpl-3.0.html

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# SPDX-License-Identifier: AGPL-3.0-only

from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    machina_ndof_sensor_driver_system_tests_dir = get_package_share_directory(
        "machina_ndof_sensor_driver_system_tests"
    )

    # Create the path to the param files
    sensor_1_service_param_file = (
        Path(machina_ndof_sensor_driver_system_tests_dir)
        / "param"
        / "machina_ndof_sensor_driver_service_sensor_1.param.yaml"
    )
    sensor_2_service_param_file = (
        Path(machina_ndof_sensor_driver_system_tests_dir)
        / "param"
        / "machina_ndof_sensor_driver_service_sensor_2.param.yaml"
    )
    sensors_client_param_file = (
        Path(machina_ndof_sensor_driver_system_tests_dir)
        / "param"
        / "machina_ndof_sensor_driver_client.param.yaml"
    )

    # Create a ComposableNodeContainer to hold the nodes
    container = ComposableNodeContainer(
        name="machina_ndof_sensor_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # Create a ComposableNode for the services
            ComposableNode(
                package="machina_ndof_sensor_driver_service",
                plugin="machina::service::ndof_sensor_driver::MachinaNDofSensorDriverServiceNode",
                name="machina_ndof_sensor_driver_service_node_1",
                parameters=[sensor_1_service_param_file],
                remappings=[
                    ("get_latest_sensor_data", "get_latest_sensor_data_1"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="machina_ndof_sensor_driver_service",
                plugin="machina::service::ndof_sensor_driver::MachinaNDofSensorDriverServiceNode",
                name="machina_ndof_sensor_driver_service_node_2",
                parameters=[sensor_2_service_param_file],
                remappings=[
                    ("get_latest_sensor_data", "get_latest_sensor_data_2"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # Create a ComposableNode for the client
            ComposableNode(
                package="machina_ndof_sensor_driver_client",
                plugin="machina::client::ndof_sensor_driver::MachinaNDofSensorDriverClientNode",
                name="machina_ndof_sensor_driver_client_node",
                parameters=[sensors_client_param_file],
                remappings=[
                    ("get_latest_sensor_data_0", "get_latest_sensor_data_1"),
                    ("get_latest_sensor_data_1", "get_latest_sensor_data_2"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    # Add the container to the launch description
    ld.add_action(container)

    return ld
