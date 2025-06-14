from lerobot.common.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader
from pydantic import BaseModel
import rclpy.node
from std_msgs.msg import Float64MultiArray
# {'shoulder_pan.pos': -4.893537542024646, 'shoulder_lift.pos': -99.66029723991507, 'elbow_flex.pos': 100.0, 'wrist_flex.pos': -99.12854030501089, 'wrist_roll.pos': -58.14679854242582, 'gripper.pos': 31.64763458401305}

class Position(BaseModel):
    pos: float

class Joints(BaseModel):
    shoulder_pan: float
    shoulder_lift: float
    elbow_flex: float
    wrist_flex: float
    wrist_roll: float
    gripper: float


teleop_config = SO101LeaderConfig(
    port="/dev/ttyACM0",
    id="leader",
)

teleop_device = SO101Leader(teleop_config)
teleop_device.connect()

rclpy.init()
node = rclpy.create_node("leader_pub")
publisher = node.create_publisher(Joints, "joints", 10)

while True:
    action = teleop_device.get_action()
    joints = Joints(
        shoulder_pan=action['shoulder_pan.pos'],
        shoulder_lift=action['shoulder_lift.pos'],
        elbow_flex=action['elbow_flex.pos'],
        wrist_flex=action['wrist_flex.pos'],
        wrist_roll=action['wrist_roll.pos'],
        gripper=action['gripper.pos'],
    )
    print(joints.model_dump_json())

    position_command = Float64MultiArray()
    position_command.data = [joints.shoulder_pan, joints.shoulder_lift, joints.elbow_flex, joints.wrist_flex, joints.wrist_roll, joints.gripper]
    publisher.publish(position_command)

# uv run python -m lerobot.teleoperate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=follower --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader
# uv run python -m lerobot.calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader


