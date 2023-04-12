# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import Command, PythonExpression, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("aspen_description"),
                    "urdf",
                    "aspen.urdf.xacro"
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("aspen_bringup"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        remappings=[
            ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
        remappings=[
            ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ]
    )

    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[{
                'angle_compensate': True,
                'scan_mode': 'Standard'
                }]
    )
 
    async_slam_node = Node(
        parameters=[
          {'params_file', '../config/mapper_params_online_async.yaml'},
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    kill_agents = ExecuteProcess(
        cmd=[[
            'sudo ',
            'pkill ',
            '-f ',
            'micro_ros_agent'
        ]],
        shell=True,
        output='screen'
    )

    micro_ros_agent_ACM0 = ExecuteProcess(
        cmd=[[
            'sudo ',
            'docker ',
            'run ', 
            '--rm ', 
            '-v ',
            '/dev:/dev ',
            '--privileged ',
            '--net=host ',
            'microros/micro-ros-agent:humble ',
            'serial ',
            '--dev ',
            '/dev/ttyACM0 ', 
        ]],
        shell=True
    )

    micro_ros_agent_ACM1 = ExecuteProcess(
        cmd=[[
            'sudo ',
            'docker ',
            'run ',
            '--rm ', 
            '-v ',
            '/dev:/dev ',
            '--privileged ',
            '--net=host ',
            'microros/micro-ros-agent:humble ',
            'serial ',
            '--dev ',
            '/dev/ttyACM1 ', 
            '-v6'
        ]],
        shell=True
    )

    return LaunchDescription([
        kill_agents,

        TimerAction(
            period=1.0,
            actions=[
            micro_ros_agent_ACM0
            ]
        ),
    
        TimerAction(
            period=2.0,
            actions=[
            micro_ros_agent_ACM1
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                control_node,
                robot_state_pub_node,
                joint_state_broadcaster_spawner,
                robot_controller_spawner,
                lidar_node
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[async_slam_node]
        )
    ])
