# Simple simulator to generate random samples.
# Adapted from
# https://github.com/Machina-Labs/robotic_hw/blob/32d90cdff30a71b680121a0889b8ecb8fb75a08c/sensor.py


# !/usr/bin/env python3

import socket
import numpy as np
from threading import Thread
import time
import argparse


class Sensor(Thread):
    def __init__(
        self,
        address: str,
        port: int,
        sampling_rate: int,
        overhead_delay: float,
        dof: int,
        num_connections: int,
    ) -> None:
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Define the server address and port
        self.server_address = (address, port)

        # Bind the server arg to the socket
        self.sock.bind(self.server_address)

        # Listen for incoming connections
        self.sock.listen(1)

        # This is an artificial delay we add as the over head
        self.overhead_delay = overhead_delay

        # The number of connections to the sensor
        self.num_connections = num_connections

        # Buffer size for reciving data. Takes in a uint64_t value
        self.buffer_size = np.dtype(np.uint64).itemsize

        self.client_address = None
        self.sensor_running = False
        self.connected = False
        self.DOF = dof
        self.sampling_rate = sampling_rate

    def connect(self) -> bool:
        # Wait for a connection
        print("Waiting for a connection")
        while self.client_address is None:
            self.connection, self.client_address = self.sock.accept()
        self.connected = True
        print("Connection from", self.client_address)
        return True

    def receive(self, buffer_size: int) -> int:
        # Read a buffer size from the socket
        recived_msg = self.connection.recv(buffer_size)
        msg = np.frombuffer(recived_msg, dtype=np.uint64)[0]
        return int(msg)

    def send(self, data: np.ndarray) -> bool:
        # Send the data to the client
        if self.connected:
            try:
                self.connection.sendall(data.tobytes(order="C"))
                return True
            except Exception as e:
                print(f"Something went wrong in sending samples to: {self.client_address}: {e}")
                return False

    def run(self):
        if self.connect():
            try:
                while True:
                    # Recive the request for the number of samples
                    sample_length = self.receive(self.buffer_size)
                    # Let's pretend we are really collecting samples
                    collection_time = (
                        (int(sample_length) / self.sampling_rate)
                        + self.overhead_delay
                        + np.random.randint(0, 100) / 100000
                    )
                    time.sleep(collection_time)

                    # Send the samples to the client
                    generated_samples = np.asarray(
                        np.random.rand(self.DOF, int(sample_length)), dtype=np.float64
                    )
                    generated_samples[generated_samples.shape[0] - 1][
                        generated_samples.shape[1] - 1
                    ] = (
                        np.sum(generated_samples)
                        - generated_samples[generated_samples.shape[0] - 1][
                            generated_samples.shape[1] - 1
                        ]
                    )
                    self.send(generated_samples)

            finally:
                # Clean up the connection
                self.connection.close()


def main(
    address: str,
    port: int,
    sampling_rate: int,
    overhead_delay: float,
    dof: int,
    num_connections: int,
) -> None:
    sensor = Sensor(address, port, sampling_rate, overhead_delay, dof, num_connections)
    sensor_thread = Thread(target=sensor.run)
    sensor_thread.daemon = True
    sensor_thread.start()

    while True:
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Machina N-DoF Sensor simulator")
    parser.add_argument("--address", type=str, required=True, help="The address of the sensor")
    parser.add_argument("--port", type=int, required=True, help="The port of the sensor")
    parser.add_argument(
        "--sampling_rate", type=int, required=True, help="The sampling rate of the sensor in Hz"
    )
    parser.add_argument(
        "--overhead_delay", type=float, required=True, help="The delay of the sensor in seconds"
    )
    parser.add_argument(
        "--dof", type=int, required=True, help="The number of degrees of freedom of the sensor"
    )
    parser.add_argument(
        "--num_connections",
        type=int,
        required=True,
        help="The number of connections to the sensor",
    )
    args = parser.parse_args()
    main(
        args.address,
        args.port,
        args.sampling_rate,
        args.overhead_delay,
        args.dof,
        args.num_connections,
    )
