# Copyright (C) 2025 Robotec.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List, cast

import rclpy
import time
import streamlit as st
from langchain_core.runnables import Runnable
from langchain_core.tools import BaseTool
from pydantic import BaseModel
from typing import Type
from rai import get_llm_model
from rai.agents.langchain import (
    ReActAgent,
    ReActAgentState,
)
from rai.communication.ros2 import ROS2Connector
from rai.frontend.streamlit import run_streamlit_app
from rai.tools.ros2 import (
    GetObjectPositionsTool,
    GetROS2ImageConfiguredTool,
    GetROS2TransformConfiguredTool,
    Nav2Toolkit,
)
from rai.tools.time import WaitForSecondsTool
from rai_open_set_vision.tools import GetGrabbingPointTool

from rai_whoami import EmbodimentInfo
from rai.tools.ros2.base import BaseROS2Tool
from rai.communication.ros2 import ROS2Message
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

# Set page configuration first
st.set_page_config(
    page_title="RAI ROSBotXL Demo",
    page_icon=":robot:",
)


# Define the input schema for the tool
class GetToBallToolInput(BaseModel):
    pass

# Define the tool
class GetToBallTool(BaseROS2Tool):
    """Tool for navigating to the ball with nav2"""
    name: str = "get_to_ball"
    description: str = "Gets to the ball pickup pose (next to it)"
    args_schema: Type[GetToBallToolInput] = GetToBallToolInput

    def _run(self) -> str:
        """Execute the ball pickup"""
        #response.payload.success = True
        response = self.connector.service_call(
            ROS2Message(
                payload={}
            ),
            target="/next_pickup_pose",
            msg_type="demo_interfaces/srv/GetPose",
        )
        print(response)
        if response.payload.success:
            navigator = BasicNavigator()
            poseWithTimestamp = PoseStamped()
            poseWithTimestamp.pose = response.payload.pose
            poseWithTimestamp.header.frame_id = "map"
            navigator.goToPose(poseWithTimestamp)
            while not navigator.isTaskComplete():
                time.sleep(0.1)
            return "Going to pose successful"
        else:
            return "Going to pose failed"

# Define the input schema for the tool
class PickBallToolInput(BaseModel):
    pass


# Define the tool
class PickBallTool(BaseROS2Tool):
    """Tool for picking the ball using a learned policy"""

    name: str = "pick_ball"
    description: str = "Picks the ball and returns to default pose with ball in gripper"
    args_schema: Type[PickBallToolInput] = PickBallToolInput

    def _run(self) -> str:
        """Execute the ball pickup"""
        # response.payload.success = True
        response = self.connector.service_call(
            ROS2Message(payload={}),
            target="/pick_ball",
            msg_type="std_srvs/srv/Trigger",
        )
        if response.payload.success:
            return "Picking ball successful"
        else:
            return "Picking ball failed " + response.payload.message


# Define the input schema for the put in basket tool
class PutInBasketToolInput(BaseModel):
    pass


# Define the put in basket tool
class PutInBasketTool(BaseROS2Tool):
    """Tool for putting the ball in the basket by moving arm to fixed position, opening gripper, and returning to center"""

    name: str = "put_in_basket"
    description: str = (
        "Moves the arm to the basket position, opens the gripper to release the ball, and returns to center position"
    )
    args_schema: Type[PutInBasketToolInput] = PutInBasketToolInput

    def _run(self) -> str:
        """Execute the ball placement sequence at the fixed basket position"""
        try:
            # Call the putballinbasket service for the fixed basket position
            response = self.connector.service_call(
                ROS2Message(payload={}),
                target="/put_ball_in_basket",
                msg_type="srd_srv/Trigger",
            )

            if response.payload.success:
                return "Ball put in basket successfully and arm returned to center"
            else:
                return f"Putting ball in basket failed: {response.payload.message if hasattr(response.payload, 'message') else 'Unknown error'}"
        except Exception as e:
            return f"Error while putting ball in basket: {str(e)}"


@st.cache_resource
def initialize_agent() -> Runnable[ReActAgentState, ReActAgentState]:
    rclpy.init()
    embodiment_info = EmbodimentInfo.from_file(
        "../../../../modules/rai/examples/embodiments/rosbotxl_embodiment.json"
    )

    connector = ROS2Connector(executor_type="multi_threaded")
    tools: List[BaseTool] = [
        GetROS2TransformConfiguredTool(
            connector=connector,
            source_frame="map",
            target_frame="base_link",
            timeout_sec=5.0,
        ),
        PickBallTool(
            connector=connector
        ),
        GetToBallTool(
            connector=connector
        ),
        PutInBasketTool(
            connector=connector
        ),
        WaitForSecondsTool()
    ]

    agent = ReActAgent(
        target_connectors={},  # empty dict, since we're using the agent in direct mode
        llm=get_llm_model("complex_model", streaming=True),
        system_prompt=embodiment_info.to_langchain(),
        tools=tools,
    ).agent
    connector.node.declare_parameter("conversion_ratio", 1.0)

    return cast(Runnable[ReActAgentState, ReActAgentState], agent)


def main():
    run_streamlit_app(
        initialize_agent(),
        "RAI ROSBotXL Demo",
        "Hi! I am a ROSBotXL robot. What can I do for you?",
    )


if __name__ == "__main__":
    main()
