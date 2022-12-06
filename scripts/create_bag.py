#!/usr/bin/env python3

import os
import shutil
from math import sin, cos

import rosbag2_py
from rclpy.serialization import serialize_message

# for video creation
import cv2
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List

from std_msgs.msg import Int32, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

from cv_bridge import CvBridge


SERIALIZATION_FORMAT = "cdr"


def get_rosbag_options(path: str):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=SERIALIZATION_FORMAT,
        output_serialization_format=SERIALIZATION_FORMAT,
    )
    return storage_options, converter_options


def create_topic(writer: rosbag2_py.SequentialWriter, topic_name: str, topic_type: str):
    topic = rosbag2_py.TopicMetadata(
        name=topic_name, type=topic_type, serialization_format=SERIALIZATION_FORMAT
    )
    writer.create_topic(topic)


def to_timestamp(milli: int):
    return milli * 1000 * 1000


def create_new_writer(bag_path: str) -> rosbag2_py.SequentialWriter:
    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)

    writer = rosbag2_py.SequentialWriter()
    writer.open(*get_rosbag_options(bag_path))
    return writer


def create_int32_bag(bag_path: str):
    writer = create_new_writer(bag_path)

    topic_name = "/number"
    create_topic(writer, topic_name, "std_msgs/msg/Int32")

    for i in range(10):
        msg = Int32(data=i)
        writer.write(topic_name, serialize_message(msg), to_timestamp(i * 1000))


def create_string_bag(bag_path: str):
    writer = create_new_writer(bag_path)

    topic_name = "/chatter"
    create_topic(writer, topic_name, "std_msgs/msg/String")

    for i in range(10):
        msg = String(data=f"Hello {i}")
        writer.write(topic_name, serialize_message(msg), to_timestamp(i * 1000))


def create_point_bag(bag_path: str):
    writer = create_new_writer(bag_path)

    dt = 0.1
    vx = 5

    topic_name = "/point"
    create_topic(writer, topic_name, "geometry_msgs/msg/Point")

    for i in range(100):
        x = i * dt * vx
        y = sin(x)
        z = cos(x)

        msg = Point(x=x, y=y, z=z)
        writer.write(
            topic_name, serialize_message(msg), to_timestamp(int(i * dt * 1000))
        )


def create_video_bag(bag_path: str):
    writer = create_new_writer(bag_path)

    dt = 0.1
    anim = Animation(dt)

    topic_name = "/image"
    create_topic(writer, topic_name, "sensor_msgs/msg/Image")

    for i in range(100):
        msg = anim.get_and_update()
        writer.write(
            topic_name,
            serialize_message(msg),
            to_timestamp(int(i * dt * 1000)),
        )


def create_mix_bag(bag_path: str):
    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)

    anim = Animation(0.1)

    writer = rosbag2_py.SequentialWriter()
    writer.open(*get_rosbag_options(bag_path))

    def wave_point(i):
        vx = 5
        x = i * 0.001 * vx
        return Point(x=x, y=sin(x), z=cos(x))

    # topic, type, dt, msg callback
    topics = [
        ("/number", "std_msgs/msg/Int32", 0.5, lambda i: Int32(data=i)),
        ("/chatter", "std_msgs/msg/String", 1, lambda i: String(data=f"Hello {i}")),
        ("/point", "geometry_msgs/msg/Point", 0.1, lambda i: wave_point(i)),
        ("/image", "sensor_msgs/msg/Image", 0.1, lambda _: anim.get_and_update()),
    ]

    for topic in topics:
        create_topic(writer, topic[0], topic[1])

    for milli in range(10000):
        # https://github.com/ros-visualization/rqt_bag/issues/136
        offset = 0
        for topic in topics:
            target_milli = int(topic[2] * 1000)
            if milli % target_milli != 0:
                continue
            msg = topic[3](milli)
            writer.write(topic[0], serialize_message(msg), to_timestamp(milli + offset))
            offset += 1


def main():
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    create_int32_bag(os.path.join(data_dir, "int32.bag"))
    create_string_bag(os.path.join(data_dir, "string.bag"))
    create_point_bag(os.path.join(data_dir, "point.bag"))
    create_video_bag(os.path.join(data_dir, "video.bag"))
    create_mix_bag(os.path.join(data_dir, "mix.bag"))


# for animation
@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: int
    color: Tuple[int, int, int]


class Field:
    def __init__(self, width: int, height: int, balls: List[Ball]):
        self.width = width
        self.height = height
        self.balls = balls

    def update(self, dt: float):
        for b in self.balls:
            nx = b.x + b.vx * dt
            ny = b.y + b.vy * dt

            # bound x
            if nx < b.radius:
                b.vx = -b.vx
                nx = b.radius + b.radius - nx
            elif nx > self.width - b.radius:
                b.vx = -b.vx
                nx = self.width - b.radius - (nx + b.radius - self.width)

            # bound y
            if ny < b.radius:
                b.vy = -b.vy
                ny = b.radius + b.radius - ny
            elif ny > self.height - b.radius:
                b.vy = -b.vy
                ny = self.height - b.radius - (ny + b.radius - self.height)
            b.x = nx
            b.y = ny

    def draw(self, img: np.ndarray):
        for b in self.balls:
            cv2.circle(img, (int(b.x), int(b.y)), b.radius, b.color, 2)


class Animation:
    def __init__(self, dt: float):
        # color is bgr8
        balls = [
            Ball(100, 50, 100, 200, 20, (255, 0, 0)),
            Ball(50, 100, 100, 200, 20, (0, 255, 0)),
            Ball(300, 200, 160, -150, 20, (0, 0, 255)),
        ]
        self.dt = dt
        self.width = 400
        self.height = 300

        self.field = Field(self.width, self.height, balls)

        self.bridge = CvBridge()

    def get_and_update(self) -> Image:
        img = np.zeros((self.height, self.width, 3), np.uint8)
        self.field.draw(img)
        cv_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.field.update(self.dt)
        return cv_img


if __name__ == "__main__":
    main()
