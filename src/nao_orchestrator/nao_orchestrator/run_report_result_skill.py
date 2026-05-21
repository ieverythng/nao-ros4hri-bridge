#!/usr/bin/env python3
"""Entrypoint for the lifecycle report-result skill action server."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from nao_orchestrator.report_result_skill_server import ReportResultSkillServer


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReportResultSkillServer()
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
