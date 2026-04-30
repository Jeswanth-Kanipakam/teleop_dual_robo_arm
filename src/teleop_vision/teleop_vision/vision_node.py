import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from std_msgs.msg import Float64MultiArray

import cv2
import numpy as np

from .Detector_Modules.HandDetectorModule import HandDetector


class TeleopMapper(Node):
    def __init__(self):
        super().__init__('teleop_mapper')

        self.left_pose_pub = self.create_publisher(
            PoseStamped,
            '/left_servo_node/target_pose',
            10
        )

        self.right_pose_pub = self.create_publisher(
            PoseStamped,
            '/right_servo_node/target_pose',
            10
        )

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/teleop_joint_angles',
            10
        )

        self.left_gripper_client = ActionClient(
            self,
            GripperCommand,
            '/left_gripper_controller/gripper_cmd'
        )

        self.right_gripper_client = ActionClient(
            self,
            GripperCommand,
            '/right_gripper_controller/gripper_cmd'
        )

        self.hand_detector = HandDetector(maxHands=2)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam")

        self.alpha = 0.2
        self.create_timer(0.03, self.timer_callback)
        self.left_j1 = 0.0
        self.left_j2 = 0.0
        self.right_j1 = 0.0
        self.right_j2 = 0.0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        frame = self.hand_detector.findHands(frame)

        if self.hand_detector.results.multi_hand_landmarks:

            for idx, hand_lms in enumerate(
                    self.hand_detector.results.multi_hand_landmarks):

                label = self.hand_detector.results.multi_handedness[
                    idx].classification[0].label

                self.hand_detector.findPosition(idx)

                center = self.hand_detector.getHandCenter()

                if center is None:
                    continue
                    
                msg = PoseStamped()
                msg.header.frame_id = "world"
                msg.header.stamp = self.get_clock().now().to_msg()

                msg.pose.position.x = 0.4

                msg.pose.position.y = (center[0] / 640 - 0.5) * 0.8

                msg.pose.position.z = (0.5 - center[1] / 480) * 0.8 + 0.5

                msg.pose.orientation.w = 1.0

                if label == "Left":
                    self.left_j1 = msg.pose.position.y
                    self.left_j2 = msg.pose.position.z
                    self.left_pose_pub.publish(msg)

                else:
                    self.right_j1 = msg.pose.position.y
                    self.right_j2 = msg.pose.position.z
                    self.right_pose_pub.publish(msg)

                aperture = self.hand_detector.findAperture()
                self.send_gripper_goal(label, aperture)

        joint_msg = Float64MultiArray()
        joint_msg.data = [
            self.left_j1,
            self.left_j2,
            self.right_j1,
            self.right_j2
        ]

        self.joint_pub.publish(joint_msg)

        cv2.imshow("Dual Arm Teleop", frame)
        cv2.waitKey(1)

    def send_gripper_goal(self, side, value):
        goal = GripperCommand.Goal()

        goal.command.position = float(value) * 0.04

        if side == "Left":
            self.left_gripper_client.send_goal_async(goal)
        else:
            self.right_gripper_client.send_goal_async(goal)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TeleopMapper()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
