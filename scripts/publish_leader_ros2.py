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

class MyNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("leader_pub")

        self.publisher = self.create_publisher(Float64MultiArray, '/base/position_controller/commands', 10)
        
        self.timer = self.create_timer(1/30.0, self.timer_callback)
        self.maxes = [0,0,0,0,0,0]
        self.mins = [0,0,0,0,0,0]

    def timer_callback(self):
        action = teleop_device.get_action()
        print(action)
        # joints = Joints(
        #     shoulder_pan=map_value(action['shoulder_pan.pos'], 0.9290807966625466 ,
        #     shoulder_lift=action['shoulder_lift.pos'] * 0.0279253,
        #     elbow_flex=action['elbow_flex.pos'] * 0.0165806,
        #     wrist_flex=action['wrist_flex.pos'] * 0.01658065,
        #     wrist_roll=action['wrist_roll.pos'] * 0.0174533,
        #     gripper=action['gripper.pos'] * 0.01919863,
        # )
        # #print(joints.model_dump_json())
        

        # position_command = Float64MultiArray()
        # position_command.data = [joints.shoulder_pan, joints.shoulder_lift, joints.elbow_flex, joints.wrist_flex, joints.wrist_roll, joints.gripper]
        # self.publisher.publish(position_command)


node = MyNode()
rclpy.spin(node)
# publisher = node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
#
# while True:
#     action = teleop_device.get_action()
#     joints = Joints(
#         shoulder_pan=action['shoulder_pan.pos'],
#         shoulder_lift=action['shoulder_lift.pos'],
#         elbow_flex=action['elbow_flex.pos'],
#         wrist_flex=action['wrist_flex.pos'],
#         wrist_roll=action['wrist_roll.pos'],
#         gripper=action['gripper.pos'],
#     )
#     print(joints.model_dump_json())
#
#     position_command = Float64MultiArray()
#     position_command.data = [joints.shoulder_pan, joints.shoulder_lift, joints.elbow_flex, joints.wrist_flex, joints.wrist_roll, joints.gripper]
#     publisher.publish(position_command)

# uv run python -m lerobot.teleoperate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=follower --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader
# uv run python -m lerobot.calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader


