#!/usr/bin/env python3
"""Ollama-backed chat skill node entrypoint."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from nao_chatbot.chat_skill_server import ChatSkillServer


def main(args=None) -> None:
    """Run the Ollama-backed chat action server node."""
    rclpy.init(args=args)
    node = ChatSkillServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
