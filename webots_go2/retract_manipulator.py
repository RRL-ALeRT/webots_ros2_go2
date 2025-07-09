import rclpy
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math


def main():
    rclpy.init()
    node = rclpy.create_node("retract_manipulator_node")

    def increment_count(_):
        nonlocal clock_msg_count
        clock_msg_count += 1

    clock_msg_count = 0

    node.create_subscription(Clock, "/clock", increment_count, 1)

    # Create an action client for the omx_joint_trajectory_controller
    arm_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/omx_joint_trajectory_controller/follow_joint_trajectory",
    )

    arm_client.wait_for_server()

    # Wait for simulation clock to initiate
    while clock_msg_count < 40:
        rclpy.spin_once(node)

    # Create a goal request to set arm joint positions
    arm_goal_msg = FollowJointTrajectory.Goal()
    arm_goal_msg.trajectory.joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
    ]
    arm_point = JointTrajectoryPoint()
    arm_point.positions = [
        0.0,
        -1.5,
        1.3,
        0.2,
    ]
    arm_goal_msg.trajectory.points.append(arm_point)

    # Send action goals
    arm_client.send_goal_async(arm_goal_msg)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
