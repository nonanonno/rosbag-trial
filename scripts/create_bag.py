#!/usr/bin/env python3

import os
import shutil

import rosbag2_py
from rclpy.serialization import serialize_message

from std_msgs.msg import Int32, String

from common import get_rosbag_options, create_topic, to_timestamp


def create_int32_bag(bag_path: str):
    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)

    writer = rosbag2_py.SequentialWriter()
    writer.open(*get_rosbag_options(bag_path))

    topic_name = "/number"
    create_topic(writer, topic_name, "std_msgs/msg/Int32")

    for i in range(10):
        msg = Int32(data=i)
        writer.write(topic_name, serialize_message(msg), to_timestamp(i * 1000))


def create_string_bag(bag_path: str):
    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)

    writer = rosbag2_py.SequentialWriter()
    writer.open(*get_rosbag_options(bag_path))

    topic_name = "/chatter"
    create_topic(writer, topic_name, "std_msgs/msg/String")

    for i in range(10):
        msg = String(data=f"Hello {i}")
        writer.write(topic_name, serialize_message(msg), to_timestamp(i * 1000))


def create_mix_bag(bag_path: str):
    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)

    writer = rosbag2_py.SequentialWriter()
    writer.open(*get_rosbag_options(bag_path))

    topics = [
        ("/number", "std_msgs/msg/Int32", lambda i: Int32(data=i)),
        ("/chatter", "std_msgs/msg/String", lambda i: String(data=f"Hello {i}")),
    ]

    for topic in topics:
        create_topic(writer, topic[0], topic[1])

    for i in range(10):
        topic = topics[i % 2]
        msg = topic[2](i)
        writer.write(topic[0], serialize_message(msg), to_timestamp(i * 1000))


def main():
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    create_int32_bag(os.path.join(data_dir, "int32.bag"))
    create_string_bag(os.path.join(data_dir, "string.bag"))
    create_mix_bag(os.path.join(data_dir, "mix.bag"))


if __name__ == "__main__":
    main()
