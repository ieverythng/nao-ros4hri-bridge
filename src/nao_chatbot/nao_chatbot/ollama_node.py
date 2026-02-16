import rclpy
from rclpy.executors import MultiThreadedExecutor

from nao_chatbot.mission_controller import MissionController
from nao_chatbot.nao_rqt_bridge import NaoRqtBridge


def main(args=None):
    """Legacy entrypoint: run bridge + mission controller together."""
    rclpy.init(args=args)
    bridge_node = NaoRqtBridge()
    mission_node = MissionController()

    bridge_node.get_logger().warn(
        'The "ollama_node" entrypoint is deprecated. Use '
        '"nao_rqt_bridge_node" and "mission_controller_node" separately.'
    )

    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    executor.add_node(mission_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        bridge_node.destroy_node()
        mission_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
