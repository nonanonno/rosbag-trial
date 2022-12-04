#!usr/bin/env python3

import os
import shutil

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List

import rosbag2_py
from rclpy.serialization import serialize_message

from cv_bridge import CvBridge

from common import get_rosbag_options, create_topic, to_timestamp


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


def create_video_bag(bag_path: str):
    balls = [
        Ball(100, 50, 100, 200, 20, (255, 0, 0)),
        Ball(50, 100, 100, 200, 20, (0, 255, 0)),
        Ball(300, 200, 160, -150, 20, (0, 0, 255)),
    ]

    dt = 0.1
    width = 400
    height = 300

    field = Field(width, height, balls)

    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)

    writer = rosbag2_py.SequentialWriter()
    writer.open(*get_rosbag_options(bag_path))

    topic_name = "/image"
    create_topic(writer, topic_name, "sensor_msgs/msg/Image")

    bridge = CvBridge()

    for i in range(100):
        img = np.zeros((height, width, 3), np.uint8)
        field.draw(img)
        writer.write(
            topic_name,
            serialize_message(bridge.cv2_to_imgmsg(img, "bgr8")),
            to_timestamp(int(i * dt * 1000)),
        )

        field.update(dt)


def main():
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    create_video_bag(os.path.join(data_dir, "video.bag"))


if __name__ == "__main__":
    main()
