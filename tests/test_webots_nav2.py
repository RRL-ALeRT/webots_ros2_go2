"""Test the `webots_go2` package with Nav2 and Moveit2."""

import os
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from launch import LaunchDescription
import launch_testing.actions
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_tests.utils import TestWebots, initialize_webots_test


@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()

    # Webots
    webots_go2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("webots_go2"),
                "launch",
                "go2_launch.py",
            )
        ),
        launch_arguments={"mode": "fast", "nav": "true"}.items(),
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("webots_go2"),
                "launch",
                "nav_launch.py",
            )
        ),
        launch_arguments={"set_initial_pose": "true", "rviz": "false"}.items(),
    )

    return LaunchDescription(
        [
            webots_go2,
            nav2,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSpot(TestWebots):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.__node = rclpy.create_node("driver_tester")
        self.wait_for_clock(self.__node, messages_to_receive=20)

    def testNav2Goal(self):
        from nav2_msgs.action import NavigateToPose

        # Delay before publishing goal position (navigation initialization can be long in the CI)
        goal_action = ActionClient(self.__node, NavigateToPose, "navigate_to_pose")
        goal_message = NavigateToPose.Goal()
        goal_message.pose.header.stamp = self.__node.get_clock().now().to_msg()
        goal_message.pose.header.frame_id = "map"
        goal_message.pose.pose.position.x = 1.2
        self.__node.get_logger().info(
            "Server is ready, waiting 10 seconds to send goal position."
        )
        goal_action.wait_for_server()
        self.wait_for_clock(self.__node, messages_to_receive=1000)
        goal_action.send_goal_async(goal_message)
        self.__node.get_logger().info("Goal position sent.")

        def on_message_received(message):
            return message.pose.pose.position.x > 0.5

        self.wait_for_messages(
            self.__node,
            Odometry,
            "/Go2/odometry",
            condition=on_message_received,
            timeout=60 * 5,
        )

    def tearDown(self):
        self.__node.destroy_node()
