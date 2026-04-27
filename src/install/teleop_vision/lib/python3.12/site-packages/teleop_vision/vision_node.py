import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import cv2
import math

from dual_arm_vision_control.Detector_Modules.HandDetectorModule import HandDetector
from dual_arm_vision_control.Detector_Modules.PoseDetectorModule import poseDetector


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # Publishers
        self.left_j1 = self.create_publisher(Float64, '/left_arm/joint1_position_controller/command', 10)
        self.right_j1 = self.create_publisher(Float64, '/right_arm/joint1_position_controller/command', 10)

        # CV modules
        self.hand = HandDetector(maxHands=2)
        self.pose = poseDetector()

        # Camera
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.03, self.loop)

    def publish(self, pub, value):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)

        frame, handedness = self.hand.findHands(frame, return_handedness=True)
        self.pose.findPose(frame)
        lm_pose = self.pose.findPosePosition(frame)

        if handedness:
            for i, h in enumerate(handedness):
                label = h.classification[0].label
                lm, frame = self.hand.findHandPosition(frame, i)

                if lm:
                    self.hand.lm_list = lm
                    frame, aperture = self.hand.findHandAperture(frame)

                    cmd = 3.0 - 0.3 * (aperture / 10)

                    if label == "Left":
                        self.publish(self.left_j1, cmd)
                    else:
                        self.publish(self.right_j1, cmd)

        if lm_pose:
            r_angle = math.radians(self.pose.findAngle(frame, 12, 14, 16))
            l_angle = math.radians(self.pose.findAngle(frame, 11, 13, 15))

            self.publish(self.right_j1, r_angle)
            self.publish(self.left_j1, l_angle)

        cv2.imshow("Dual Arm", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()