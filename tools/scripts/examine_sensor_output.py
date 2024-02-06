from mcap_ros2.reader import read_ros2_messages
from pathlib import Path
from tqdm import tqdm
import argparse
import numpy as np
import pandas as pd
import pyperclip


def get_difference_in_time(msg1, msg2):
    diff_ns = (msg2.nanosec + (msg2.sec * 1e9)) - (msg1.nanosec + (msg1.sec * 1e9))
    return diff_ns / 1e9


def main(bagfile, topic):
    if not bagfile.exists():
        print(f"Bagfile {bagfile} does not exist")
        return
    if not bagfile.suffix == ".mcap":
        print(f"Bagfile {bagfile} is not an MCAP file")
        return
    mcap_summary = []
    prev_msg = None
    for msg in tqdm(read_ros2_messages(source=str(bagfile), topics=[topic])):
        row_info = {}
        if prev_msg:
            row_info["publisher_time_diff_sec"] = get_difference_in_time(
                prev_msg.ros_msg.stamp, msg.ros_msg.stamp
            )
        else:
            prev_msg = msg
            continue
        for i in range(len(msg.ros_msg.sensors)):
            row_info[f"sensor_{i}_time_diff_sec"] = get_difference_in_time(
                prev_msg.ros_msg.sensors[i].header.stamp, msg.ros_msg.sensors[i].header.stamp
            )
            row_info[f"sensor_{i}_average_deviation"] = np.abs(
                np.array(prev_msg.ros_msg.sensors[i].data.data)
                - np.array(msg.ros_msg.sensors[i].data.data)
            ).mean()
            row_info[f"sensor_{i}_data_valid"] = np.uint8(
                np.isclose(
                    np.sum(msg.ros_msg.sensors[i].data.data)
                    - msg.ros_msg.sensors[i].data.data[-1],
                    msg.ros_msg.sensors[i].data.data[-1],
                )
            )
        mcap_summary.append(row_info)
        prev_msg = msg
    mcap_summary = pd.DataFrame(mcap_summary)
    print(mcap_summary.describe())
    pyperclip.copy(mcap_summary.describe().to_markdown())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Examine sensor output")
    parser.add_argument(
        "-i", "--bagfile", help="ROS2 bagfile to examine", type=Path, required=True
    )
    parser.add_argument("-t", "--topic", help="Topic to examine", type=str, required=True)
    args = parser.parse_args()

    main(args.bagfile, args.topic)
