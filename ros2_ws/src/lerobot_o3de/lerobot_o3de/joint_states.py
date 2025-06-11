import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# This node converts the joint_states messages to position commands understood by O3DE.
class PositionCommandPublisher(Node):
    def __init__(self):
        super().__init__('position_command_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'position_controller/commands', 10)
        self.i = 0
    
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.get_logger().info('PositionCommandPublisher node has been started.')

    def joint_states_callback(self, msg):
        position_command = Float64MultiArray()
        position_command.data = msg.position # Example position command
        self.publisher_.publish(position_command)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    position_command_publisher = PositionCommandPublisher()

    rclpy.spin(position_command_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()