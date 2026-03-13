#!/usr/bin/env python3
"""Entrypoint for the lifecycle NAO say skill."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from nao_say_skill.skill_impl import NaoSaySkill


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NaoSaySkill()
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
