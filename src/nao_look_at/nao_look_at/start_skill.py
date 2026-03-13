#!/usr/bin/env python3
"""Entrypoint for the lifecycle NAO look_at scaffold."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from nao_look_at.skill_impl import NaoLookAtSkill


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NaoLookAtSkill()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
