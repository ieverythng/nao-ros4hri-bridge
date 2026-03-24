#!/usr/bin/env python3
"""Entrypoint for the NAO implementation of interaction_skills/look_at."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from nao_look_at.skill_impl import NaoLookAtSkill


def main(args=None) -> None:
    """Launch the NAO implementation of the upstream look_at contract."""
    rclpy.init(args=args)

    # Keep this bootstrap obvious: this package is the NAO-facing runtime
    # implementation behind the upstream interaction_skills contract.
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
