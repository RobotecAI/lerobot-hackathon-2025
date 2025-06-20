import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import time


BASKET_POSITION = [
  0.012269669900000002,
  -0.7696212679999999,
  -1.65806,
  -1.351042143,
  -0.009424781999999965,
  0.01
]  # 


# BASKET_POSITION = [
#   -0.0603011515,
#   -0.558506,
#   -1.65806,
#   -1.74533,
#   -0.009424781999999965,
#   0.01036724400000022,
# ]  # 


# BASKET_POSITION = [
#     1.75,
#     0.5,
#     0.76,
#     1.57,
#     -1.5,
#     0.0,
# ]  # to be calibrated


class PutBallInBasketService(Node):
    def __init__(self):
        super().__init__("put_ball_in_basket_service")

        # Publisher for joint position commands
        self.position_publisher = self.create_publisher(
            Float64MultiArray, "/base/position_controller/commands", 10
        )

        # Service for putting ball in basket
        self.put_ball_in_basket_service = self.create_service(
            Trigger, "/put_ball_in_basket", self.put_ball_in_basket_callback
        )

        # Define joint positions (these will be set later based on calibration)
        # Assuming 6 DOF arm + gripper
        self.center_position = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # Center/home position
        self.basket_position = BASKET_POSITION

        self.gripper_open_position = self.basket_position.copy()
        self.gripper_open_position[-1] = 1.5

        # Movement parameters
        self.movement_duration = 2.0  # seconds to complete each movement
        self.step_delay = 0.1  # delay between position updates

        self.get_logger().info("PutBallInBasketService node has been started.")

    def put_ball_in_basket_callback(self, request, response):
        """Service callback to put ball in basket"""
        self.get_logger().info("Received putballinbasket request")

        try:
            # Step 1: Move to basket position
            self.get_logger().info("Moving arm to basket position...")
            self.move_to_position(self.basket_position)
            time.sleep(0.5)  # Brief pause at basket position

            # Step 2: Open gripper to release ball
            self.get_logger().info("Opening gripper to release ball...")
            basket_with_open_gripper = self.basket_position.copy()
            basket_with_open_gripper[-1] = self.gripper_open_position[
                -1
            ]  # Open gripper
            self.move_to_position(basket_with_open_gripper)
            time.sleep(1.0)  # Wait for ball to drop

            # Step 3: Return to center position
            self.get_logger().info("Returning to center position...")
            self.move_to_position(self.center_position)

            response.success = True
            response.message = "Ball put in basket successfully"
            self.get_logger().info(
                "Put ball in basket operation completed successfully"
            )

        except Exception as e:
            response.success = False
            response.message = f"Failed to put ball in basket: {str(e)}"
            self.get_logger().error(f"Put ball in basket operation failed: {str(e)}")

        return response

    def move_to_position(self, target_position: list[float]):
        """Move joints smoothly to target position"""
        # Create and publish position command
        position_command = Float64MultiArray()
        position_command.data = target_position

        # Publish the target position
        self.position_publisher.publish(position_command)

        # Wait for movement to complete
        time.sleep(self.movement_duration)

    def interpolate_movement(
        self, start_pos: list[float], end_pos: list[float], steps: int = 20
    ):
        """Smoothly interpolate between start and end positions"""
        for i in range(steps + 1):
            alpha = i / steps
            interpolated_pos = [
                start + alpha * (end - start) for start, end in zip(start_pos, end_pos)
            ]

            position_command = Float64MultiArray()
            position_command.data = interpolated_pos
            self.position_publisher.publish(position_command)

            time.sleep(self.step_delay)


def main(args=None):
    rclpy.init(args=args)

    put_ball_in_basket_service = PutBallInBasketService()

    # Use MultiThreadedExecutor to handle service calls properly
    executor = MultiThreadedExecutor()
    executor.add_node(put_ball_in_basket_service)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        put_ball_in_basket_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
