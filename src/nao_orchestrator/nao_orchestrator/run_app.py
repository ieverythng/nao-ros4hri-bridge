#!/usr/bin/env python3
"""Entrypoint for the NAO orchestrator lifecycle node."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from nao_orchestrator.orchestrator import NaoOrchestrator


def main(args=None) -> None:
    """Launch the lifecycle orchestrator with a multithreaded executor."""
    rclpy.init(args=args)

    # The orchestrator fans out to several skills and bridge services, so the
    # executor bootstrap is kept explicit here for easier review.
    node = NaoOrchestrator()
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


if __name__ == "__main__":
    main()
