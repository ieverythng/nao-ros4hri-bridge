#!/usr/bin/env python3
"""Entrypoint for the NAO orchestrator lifecycle node."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from nao_orchestrator.orchestrator import NaoOrchestrator


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NaoOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
