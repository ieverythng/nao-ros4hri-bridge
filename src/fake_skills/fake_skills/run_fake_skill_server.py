"""Entrypoint for fake skill action server node."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from fake_skills.action_server import FakeSkillActionServer


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeSkillActionServer()
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


if __name__ == '__main__':
    main()
