import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ROSPublisher(Node):
    def __init__(self):
        super().__init__('dual_arm_publisher')

        # LEFT ARM
        self.left_j1_pub = self.create_publisher(
            Float64, '/left_arm/joint1_position_controller/command', 10)
        self.left_j2_pub = self.create_publisher(
            Float64, '/left_arm/joint2_position_controller/command', 10)

        # RIGHT ARM
        self.right_j1_pub = self.create_publisher(
            Float64, '/right_arm/joint1_position_controller/command', 10)
        self.right_j2_pub = self.create_publisher(
            Float64, '/right_arm/joint2_position_controller/command', 10)

    def publish_left(self, j1, j2):
        msg1 = Float64()
        msg2 = Float64()

        msg1.data = float(j1)
        msg2.data = float(j2)

        self.left_j1_pub.publish(msg1)
        self.left_j2_pub.publish(msg2)

    def publish_right(self, j1, j2):
        msg1 = Float64()
        msg2 = Float64()

        msg1.data = float(j1)
        msg2.data = float(j2)

        self.right_j1_pub.publish(msg1)
        self.right_j2_pub.publish(msg2)