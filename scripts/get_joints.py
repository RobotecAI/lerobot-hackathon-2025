from lerobot.common.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader
from lerobot.common.robots.so101_follower import SO101FollowerConfig, SO101Follower
from pydantic import BaseModel
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


robot_config = SO101FollowerConfig(
    port="/dev/ttyACM1",
    id="follower",
)

teleop_config = SO101LeaderConfig(
    port="/dev/ttyACM0",
    id="leader",
)

robot = SO101Follower(robot_config)
teleop_device = SO101Leader(teleop_config)
robot.connect()
teleop_device.connect()

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
    robot.send_action(action)

# uv run python -m lerobot.teleoperate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=follower --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader
# uv run python -m lerobot.calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=leader


# {"shoulder_pan":-0.7844602166604489,"shoulder_lift":-24.416135881104026,"elbow_flex":-33.33333333333334,"wrist_flex":99.2156862745098,"wrist_roll":-54.450806871421136,"gripper":55.301794453507334}