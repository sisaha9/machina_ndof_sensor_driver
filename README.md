# Machina NDoF Sensor Driver

My solution to the challenged posted by Machina Labs [here](https://github.com/Machina-Labs/robotic_hw) with 2 days spent on it

Documentation website can be seen [here](https://sisaha9.github.io/machina_ndof_sensor_driver/)

## Table of Contents

- [Machina NDoF Sensor Driver](#machina-ndof-sensor-driver)
  - [Design Methodology / Description of each node / Future Improvements](#design-methodology--description-of-each-node--future-improvements)
  - [Build and Run](#build-and-run)
  - [Results](#results)

## Design Methodology / Description of each node / Future Improvements

There are 6 packages in this repo. They are

1. `machina_ndof_sensor_driver`
    - A Metapackage for the rest of the packages in this driver. Can be used as a direct depend to build all other packages in this repo
    - It also additionally includes a depend on `rmw_cyclonedds_cpp`. It isn't a strict requirement but it's what I have found most convenient to use in ROS 2 especially with services. Feel free to comment it out / remove it if you don't need / want it (Note the Dockerfile automatically uses CycloneDDS with a config)
2. `machina_tcp_interface`
    - Contains a library implementation of a TCP Socket that can be used by other nodes / libraries. Uses `arpa` and `sys` libraries to connect to a TCP socket, read and send buffers as well as check status information of the connection
    - Thought about exposing it as a implementation similar to [transport_drivers](https://github.com/ros-drivers/transport_drivers). It would be useful for being able to record network packets and replay later to test out. But decided against it for the sake of time.
    - Need to add unit tests but ran out of time
3. `machina_ndof_sensor_driver_msgs`
    - Contains custom messages and services used by the driver
    - We wrap the `example_interfaces/msg/Float64MultiArray` with a `std_msgs/Header` to form `SensorOutput.msg`. I believe it's important to record the sensor's frame ID and time it was collected for downstream processing later
    - We then create a `JointSensorOutput.msg` that has it's own `builtin_interfaces/Time`. In this case each sensor still retains it's own timestamp and frame ID. But we are also recording the time the message was generated to be used downstream as an indication for whether or not the data is frozen
    - Finally, there is a custom service called `GetLatestSensorData.srv`. It takes an empty request and sends back a boolean flag if any data was filled, a timestamp for when the data was collected and the data itself. The latter 2 are combined together through the `SensorOutput.msg`
        - I thought about letting the client send custom parameters here like number of degrees of freedom and sample size. However, given the high frequency requirements I decided that dynamically allocating memory to accomodate these custom requests would affect the frequency and hence didn't go through with it. But realistically we could decide on a max for each of those parameters and not allow the client to exceed it thus allowing us to dynamically change the kind of data we are retrieving. Again, for the sake of time decided not to go through with it
    - Need to add unit tests but ran out of time
4. `machina_ndof_sensor_driver_service`
    - A ROS 2 Service Implementation that connects via the `machina_tcp_client` to a sensor, constantly polls it for data and exposes that data for other nodes to receive through the ROS 2 Service
    - A thread is spawned to read sensor data continuously and store it. When a service request is received, the latest sensor data is served. Since this is concurrent, we have defined 2 modes of operation:
        - Lock-free using TACO from [iceoryx](https://iceoryx.io/latest/advanced/iceoryx_hoofs/#concurrent)
        - Using shared_mutex
    - You can toggle between these modes by setting -DENABLE_LOCKFREE to ON/OFF. ON for lock-free and OFF for shared_mutex. More experiments and data needs to be collected but right now TACO enables a significant speedup but it blocks access to the data often at high frequencies. Thinking of swapping to a 2 element container in the future so that the producer can always write to the element not being read. If one of the element is being written to (can use `std::atomic` to check) then the consumer can read from the other element. This way we can avoid blocking. Ommited for the sake of time
    - The node also uses the [Lifecycle Design Pattern](https://design.ros2.org/articles/node_lifecycle.html). With this, we can separate out portions of the stack where memory allocation is happening from the active runtime loop. We also have a lot of control in terms of synchronizing several lifecycle nodes and sending transitions as needed. We are able to activate and deactivate this node as needed without having to restart the node
    - The node also uses the [Composition Design Pattern](https://docs.ros.org/en/iron/Concepts/Intermediate/About-Composition.html) Ideally, this would mean there is a zero copy transfer of data between nodes in the same component container. However, I am not sure if that applies in the case of service-client relationship.
    - Another item ommited for the sake of time was to use [TypeAdapters](https://ros.org/reps/rep-2007.html) to reduce the number of memory copies. If we have 2 nodes in the same container and use type adapters, we could get the data, send it in the exact same format it was received in to the other node and it can use it directly. This would reduce the number of copies even further.
    - Need to add unit tests but ran out of time.
    - Need to add a telemetry publisher with details on time taken in some of the operations but ran out of time.
5. `machina_ndof_sensor_driver_client`
    - A ROS 2 Client Implementation that connects to several sensor services from `machina_ndof_sensor_driver_service`, combine the data into 1 topic and publish it out
    - Split into `MachinaNDoFSensorDriverClient` and `MachinaNDoFSensorDriverClientNode`. The `MachinaNDoFSensorDriverClient` is responsible for maintaining thread safety within it's class, spawn a thread to send requests to a single sensor's service and store the latest data to be accessed by `MachinaNDoFSensorDriverClientNode` later. The `MachinaNDoFSensorDriverClientNode` is responsible for periodically polling each instance of `MachinaNDoFSensorDriverClient` for the latest data and then publishing out the joint output
    - Like the service implementation this also exposes 2 methods to handle data concurrency, the `TACO` from iceoryx and a `std::shared_mutex`
    - One unique portion of the Client implementation, unlike the Service is that there are multiple type of callbacks being handled by the ROS executor. In the Service implementation, only the service callback was sent to the ROS 2 executor so it was fine to run using a `StaticSingleThreadedExecutor`. In the client implementation we have 1 timer callback and `M` sensor client callbacks (where `M` is the number of sensors) that are all being handled by the ROS 2 executor. Due to the thread safety modes implemented above, they can all be handled concurrently. Thus, we switch the executor for the client implementation to a `MultiThreadedExecutor` and have each of the client callbacks and the timer callbacks be on their own callback group so that the executor runs them in parallel
    - The points off `LifecycleNode`, `Composition` and `TypeAdapters` from the service implementation still applies here.
    - Need to add unit tests but ran out of time.
    - Need to add a telemetry publisher with details on time taken in some of the operations but ran out of time.
6. `machina_ndof_sensor_driver_system_tests`
    - Finally we have the system tests package. This was suppose to leverage [launch_testing](https://index.ros.org/p/launch_testing/#iron) to set up the sensor simulation, service implemention, client implementation and a subscriber to the telemetries and joint sensor output to validate them. Ran out of time for this
    - Rignt now contains parameter files and launch files that can be used in conjunction with a `tmux` config to record a ROS bag for post analysis. There is a script in `tools/scripts/examine_sensor_output.py` which will print out some stats for you as well as copy a Markdown representation into your clipboard
    - Also contains a sensor_simulator under `tests`. This was adapted from the base one provided by Machina. I modified it to accept a `uint64` for sample size instead of string, to replace the last element in the data with the sum of all elements before it so that it could act as a CRC of sorts, specify some of the dtypes in Numpy arrays and accept arguments from the command line.

Outside of these core packages there are the following:

1. Docker
    - A Dockerfile that builds a development environment for you to work with (under `tools/image`)
2. DevContainer
    - A DevContainer configuration for VSCode (under `.devcontainer`)
3. Github Actions
    - A CI file under `.github/workflows` to build and test the driver implementation as well as generate the documentation page
4. Makefile
    - A Makefile with some convenient commands

## Build and Run

You can build the Docker container by using

```bash
make build-docker-cpu-iron IMG_NAME=machina_3dof_sensor_driver_dev_image:latest
```

The IMG_NAME can be swapped with anything you find convenient

To build the stack, just type

```bash
make
```

This will build the stack with locks enabled. If you want to turn it off

```bash
make build-lockfree
```

Both the above `make` targets build with `Release`. If you want a `Debug` build then

```bash
make build-debug
make build-debug-lockfree
```

will build with `Debug` enabled.

If you want to go for a fresh build

```bash
make clean
```

And then type the `make` command based off how you want to build

If you want to generate the docs

```bash
make docs
```

If you want to delete the docs

```bash
make clean-docs
```

If you want to delete everything

```bash
make purge
```

If you want to run tests (right now just runs style tests)

```bash
make test
```

If you want to clean up the results

```bash
make clean-test
```

If you want to reformat your code

```bash
make reformat PATHS=src
```

where you can change the `PATHS` as needed

All the build and test target also accept a `PACKAGES=` in case you want to run up to specific packages. For example

```bash
make <target> PACKAGES="machina_tcp_interface machina_ndof_sensor_driver_msgs"
```

If you want to just run everything at once there is a reference `tmux` config provided under `tools/tmux_configs`. You can run it by

```bash
tmuxp load tools/tmux_configs/machina_sensor_simulation.yaml
```

It will spawn 2 sensors, run a launch file with 2 services to connect to each sensor and the client. It will also run the lifecycle transitions for you and after a minute deactivate them. Finally, it will record a MCAP for you. Make sure to close the MCAP using Ctrl + C before you kill the session so that it can close properly

Once you have that bag you can run

```bash
python3 tools/scripts/examine_sensor_output.py -i <path_to_mcap> -t /joint_sensor_output
```

It will print out some summary statistics and copy a Markdown version of the statistics to your clipboard. You will see some examples later in the next section

## Results

The data for these comparisons has been uploaded [here](https://drive.google.com/drive/folders/1HGUMTZRIGg4t_TtFQf86Z-jE6Fw97zQu?usp=sharing)

The summary with locks enabled was

|       |   publisher_time_diff_sec |   sensor_0_time_diff_sec |   sensor_0_average_deviation |   sensor_0_data_valid |   sensor_1_time_diff_sec |   sensor_1_average_deviation |   sensor_1_data_valid |
|:------|--------------------------:|-------------------------:|-----------------------------:|----------------------:|-------------------------:|-----------------------------:|----------------------:|
| count |            1565           |             1565         |                  1565        |                  1565 |             1565         |                  1565        |                  1565 |
| mean  |               0.0369636   |                0.0369822 |                     0.177633 |                     1 |                0.0369916 |                     0.177166 |                     1 |
| std   |               0.0189857   |                0.0361484 |                     0.173388 |                     0 |                0.0403624 |                     0.173364 |                     0 |
| min   |               0.000128512 |                0         |                     0        |                     1 |                0         |                     0        |                     1 |
| 25%   |               0.022933    |                0         |                     0        |                     1 |                0         |                     0        |                     1 |
| 50%   |               0.0367444   |                0.0623309 |                     0.319733 |                     1 |                0.0438513 |                     0.32158  |                     1 |
| 75%   |               0.0501245   |                0.0722235 |                     0.345725 |                     1 |                0.0545754 |                     0.34565  |                     1 |
| max   |               0.109515    |                0.0800883 |                     0.403066 |                     1 |                0.157095  |                     0.393948 |                     1 |

The summary with locks disabled (so lock free) was

|       |   publisher_time_diff_sec |   sensor_0_time_diff_sec |   sensor_0_average_deviation |   sensor_0_data_valid |   sensor_1_time_diff_sec |   sensor_1_average_deviation |   sensor_1_data_valid |
|:------|--------------------------:|-------------------------:|-----------------------------:|----------------------:|-------------------------:|-----------------------------:|----------------------:|
| count |            13063          |           13063          |                13063         |                 13063 |            13063         |                13063         |                 13063 |
| mean  |                0.00444128 |               0.00444002 |                    0.0212402 |                     1 |                0.0044335 |                    0.0312561 |                     1 |
| std   |                0.00160622 |               0.017368   |                    0.083015  |                     0 |                0.014126  |                    0.0993279 |                     0 |
| min   |                7.0144e-05 |               0          |                    0         |                     1 |                0         |                    0         |                     1 |
| 25%   |                0.00496973 |               0          |                    0         |                     1 |                0         |                    0         |                     1 |
| 50%   |                0.00505574 |               0          |                    0         |                     1 |                0         |                    0         |                     1 |
| 75%   |                0.00508288 |               0          |                    0         |                     1 |                0         |                    0         |                     1 |
| max   |                0.0500941  |               0.0818102  |                    0.397955  |                     1 |                0.061439  |                    0.396857  |                     1 |

Couple of observations

- Both cases had the `sensor_i_data_valid` true at all times during publish
- The `publisher_time_diff_sec` is basically a measure of how often the timer callback was triggered in the client implementation. In these experiments they were set to poll at 500Hz.
  - With locks enabled, the timer ran at a frequency of about 27.05Hz. On the other hand, without locks we were able to run at 225.16Hz
  - The main bottleneck here would be the contention b/w the client threads trying to get the latest data from the service while the publisher was trying to access the latest data to publish it out
- The `sensor_i_time_diff_sec` is a measure of how often the sensor data was updated in b/w service calls. We can see that it has been similarly throttled like the `publisher_time_diff_sec`. However, in the lock free case we see a median of 0 which means that very often we were sending out outdated data even though it was polling at 2000Hz from the TCP client
- Similarly, in `sensor_i_average_deviation` we can see that the data deviation was much lower in the lock free case than in the lock case. Again the median hits 0 here for the lock free case which is a sign that we are constantly receiving outdated data

From the debugging I did, the issue still seems to be in thread contention. I believe as the number of threads/sensors scale up the lock free implementation will do a better job of producing data reliably. I do think the `2 element container` method mentioned earlier would significantly reduce the contention. But it needs to be tested
