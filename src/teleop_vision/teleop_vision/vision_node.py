import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, Trigger

from Detector_Modules.HandDetectorModule import HandDetector
from Detector_Modules.PoseDetectorModule import PoseDetector

import cv2
import numpy as np


def normalize_camera_device(camera_device):
    if isinstance(camera_device, str) and camera_device.isdigit():
        return int(camera_device)
    return camera_device


class TeleopMapper(Node):
    def __init__(self):
        super().__init__('teleop_mapper')

        self.left_jog_pub = self.create_publisher(
            JointJog,
            '/left_servo_node/delta_joint_cmds',
            10
        )

        self.right_jog_pub = self.create_publisher(
            JointJog,
            '/right_servo_node/delta_joint_cmds',
            10
        )

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/teleop_joint_angles',
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
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

        self.left_servo_start_clients = [
            self.create_client(Trigger, '/left_servo_node/start_servo'),
            self.create_client(Trigger, '/left_servo_node/servo_node/start_servo'),
        ]
        self.right_servo_start_clients = [
            self.create_client(Trigger, '/right_servo_node/start_servo'),
            self.create_client(Trigger, '/right_servo_node/servo_node/start_servo'),
        ]
        self.left_servo_reset_clients = [
            self.create_client(Empty, '/left_servo_node/reset_servo_status'),
            self.create_client(Empty, '/left_servo_node/servo_node/reset_servo_status'),
        ]
        self.right_servo_reset_clients = [
            self.create_client(Empty, '/right_servo_node/reset_servo_status'),
            self.create_client(Empty, '/right_servo_node/servo_node/reset_servo_status'),
        ]
        self.left_servo_started = False
        self.right_servo_started = False
        self.left_servo_reset = False
        self.right_servo_reset = False

        self.hand_detector = HandDetector(maxHands=2)
        self.pose_detector = PoseDetector()

        self.declare_parameter('camera_device', '/dev/video0')
        camera_device = self.get_parameter(
            'camera_device'
        ).get_parameter_value().string_value

        self.cap = cv2.VideoCapture(normalize_camera_device(camera_device))

        if not self.cap.isOpened():
            self.get_logger().error(
                f"Could not open webcam: {camera_device}"
            )

        self.alpha = 0.2
        self.dead_zone = 0.02
        self.joint_kp = 1.4
        self.max_joint_velocity = 0.9

        self.create_timer(0.03, self.timer_callback)

        self.left_joint_cmds = [0.0] * 6
        self.right_joint_cmds = [0.0] * 6
        self.current_joint_positions = {}

    def timer_callback(self):
        self.start_servo_nodes()

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        frame = self.pose_detector.findPose(frame)
        self.pose_detector.findPosition(frame)
        frame = self.hand_detector.findHands(frame)
        seen_left = False
        seen_right = False

        if (
            self.hand_detector.results
            and self.hand_detector.results.multi_hand_landmarks
        ):

            for idx, hand_lms in enumerate(
                    self.hand_detector.results.multi_hand_landmarks):

                label = self.hand_detector.results.multi_handedness[
                    idx].classification[0].label

                self.hand_detector.findPosition(frame, idx)

                center = self.hand_detector.getHandCenter()

                if center is None:
                    continue

                msg = JointJog()
                msg.header.frame_id = "base"
                msg.header.stamp = self.get_clock().now().to_msg()
                side = label.lower()
                velocities, targets = self.compute_arm_velocities(
                    side,
                    frame.shape
                )

                cv2.putText(
                    frame,
                    (
                        f"{label}: j1={velocities[0]:+.2f} "
                        f"j2={velocities[1]:+.2f} "
                        f"j3={velocities[2]:+.2f} "
                        f"j4={velocities[3]:+.2f} "
                        f"j5={velocities[4]:+.2f} "
                        f"j6={velocities[5]:+.2f}"
                    ),
                    (20, 35 + 30 * idx),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                if label == "Left":
                    msg.joint_names = self.arm_joint_names('left')
                    msg.velocities = velocities
                    self.left_joint_cmds = velocities
                    seen_left = True
                    self.left_jog_pub.publish(msg)
                    self.draw_arm_targets(frame, 'left', targets)

                else:
                    msg.joint_names = self.arm_joint_names('right')
                    msg.velocities = velocities
                    self.right_joint_cmds = velocities
                    seen_right = True
                    self.right_jog_pub.publish(msg)
                    self.draw_arm_targets(frame, 'right', targets)

                aperture = self.hand_detector.findAperture()
                self.send_gripper_goal(label, aperture)

        if not seen_left:
            self.left_jog_pub.publish(
                self.zero_jog(self.arm_joint_names('left'))
            )
            self.left_joint_cmds = [0.0] * 6
        if not seen_right:
            self.right_jog_pub.publish(
                self.zero_jog(self.arm_joint_names('right'))
            )
            self.right_joint_cmds = [0.0] * 6

        joint_msg = Float64MultiArray()
        joint_msg.data = self.left_joint_cmds + self.right_joint_cmds

        self.joint_pub.publish(joint_msg)

        # Display
        cv2.imshow("Dual Arm Teleop", frame)
        cv2.waitKey(1)

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def compute_arm_velocities(self, side, frame_shape):
        targets = self.compute_arm_targets(side, frame_shape)
        joint_names = self.arm_joint_names(side)
        velocities = []

        for name, target in zip(joint_names, targets):
            current = self.current_joint_positions.get(name)
            if current is None:
                velocities.append(0.0)
                continue

            error = target - current
            if abs(error) < 0.03:
                velocities.append(0.0)
            else:
                velocities.append(
                    float(np.clip(
                        error * self.joint_kp,
                        -self.max_joint_velocity,
                        self.max_joint_velocity
                    ))
                )

        return velocities, targets

    def compute_arm_targets(self, side, frame_shape):
        arm_points = self.pose_detector.getArmPoints(side)
        fallback = self.get_current_targets(side)
        if arm_points is None:
            return fallback

        height, width = frame_shape[:2]
        shoulder, elbow, wrist = arm_points
        upper = elbow - shoulder
        forearm = wrist - elbow
        full = wrist - shoulder

        upper_len = np.linalg.norm(upper)
        forearm_len = np.linalg.norm(forearm)
        reach_len = np.linalg.norm(full)
        total_len = max(upper_len + forearm_len, 1e-6)
        reach = np.clip(reach_len / total_len, 0.0, 1.0)

        elbow_angle = self.angle_between(-upper, forearm)
        wrist_center_x = wrist[0] / max(width, 1)
        wrist_center_y = wrist[1] / max(height, 1)
        shoulder_center_y = shoulder[1] / max(height, 1)

        j1 = np.interp(wrist_center_x, [0.15, 0.85], [-1.2, 1.2])

        vertical_raise = np.clip(
            (shoulder_center_y - wrist_center_y) / 0.45,
            -1.0,
            1.0
        )
        j2 = np.interp(vertical_raise, [-1.0, 1.0], [0.35, 1.65])
        j2 += np.interp(reach, [0.45, 1.0], [-0.25, 0.35])
        j2 = np.clip(j2, 0.15, 2.4)

        j3 = np.interp(elbow_angle, [55.0, 175.0], [-2.1, -0.25])
        j3 = np.clip(j3, -2.5, -0.15)

        j4 = fallback[3]
        j5 = fallback[4]
        j6 = fallback[5]
        landmarks = self.hand_detector.lm_list
        if len(landmarks) >= 21:
            points = {idx: np.array([x, y], dtype=float)
                      for idx, x, y in landmarks}
            wrist = points[0]
            index_mcp = points[5]
            middle_tip = points[12]
            pinky_mcp = points[17]
            thumb_tip = points[4]
            index_tip = points[8]

            palm_width = np.linalg.norm(index_mcp - pinky_mcp)
            finger_length = np.linalg.norm(middle_tip - wrist)
            pinch_width = np.linalg.norm(index_tip - thumb_tip)
            eps = 1e-6

            j4 = np.clip(
                (pinky_mcp[1] - index_mcp[1]) / (palm_width + eps),
                -1.0,
                1.0
            ) * 0.9
            j5 = np.clip(
                (wrist[1] - middle_tip[1]) / (finger_length + eps) - 0.7,
                -1.0,
                1.0
            ) * 0.9
            j6 = np.clip(
                (index_tip[0] - thumb_tip[0]) / (pinch_width + eps),
                -1.0,
                1.0
            ) * 1.2

        return [
            float(j1),
            float(j2),
            float(j3),
            float(j4),
            float(j5),
            float(j6),
        ]

    def get_current_targets(self, side):
        return [
            self.current_joint_positions.get(name, 0.0)
            for name in self.arm_joint_names(side)
        ]

    def angle_between(self, a, b):
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        if denom < 1e-6:
            return 90.0
        cos_angle = np.clip(np.dot(a, b) / denom, -1.0, 1.0)
        return np.degrees(np.arccos(cos_angle))

    def draw_arm_targets(self, frame, side, targets):
        points = self.pose_detector.getArmPoints(side)
        if points is None:
            return

        shoulder, elbow, wrist = points
        for point in (shoulder, elbow, wrist):
            cv2.circle(frame, tuple(point.astype(int)), 6, (255, 255, 0), -1)
        cv2.line(frame, tuple(shoulder.astype(int)), tuple(elbow.astype(int)),
                 (255, 255, 0), 3)
        cv2.line(frame, tuple(elbow.astype(int)), tuple(wrist.astype(int)),
                 (255, 255, 0), 3)

    def arm_joint_names(self, side):
        return [f'{side}_joint{i}' for i in range(1, 7)]

    def zero_jog(self, joint_names):
        msg = JointJog()
        msg.header.frame_id = "base"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = joint_names
        msg.velocities = [0.0] * len(joint_names)
        return msg

    def start_servo_nodes(self):
        self.left_servo_started = self.start_servo_node(
            self.left_servo_start_clients,
            self.left_servo_started
        )
        self.right_servo_started = self.start_servo_node(
            self.right_servo_start_clients,
            self.right_servo_started
        )
        self.left_servo_reset = self.reset_servo_status(
            self.left_servo_reset_clients,
            self.left_servo_reset
        )
        self.right_servo_reset = self.reset_servo_status(
            self.right_servo_reset_clients,
            self.right_servo_reset
        )

    def start_servo_node(self, clients, already_started):
        if already_started:
            return True

        for client in clients:
            if client.service_is_ready():
                self.get_logger().info(
                    f"Starting Servo through {client.srv_name}"
                )
                future = client.call_async(Trigger.Request())
                future.add_done_callback(self.log_servo_start_result)
                return True

        return False

    def reset_servo_status(self, clients, already_reset):
        if already_reset:
            return True

        for client in clients:
            if client.service_is_ready():
                self.get_logger().info(
                    f"Resetting Servo status through {client.srv_name}"
                )
                client.call_async(Empty.Request())
                return True

        return False

    def log_servo_start_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Servo start service failed: {exc}")
            return

        if not result.success:
            self.get_logger().warn(f"Servo start rejected: {result.message}")

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
