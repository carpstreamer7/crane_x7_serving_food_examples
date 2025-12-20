# Copyright 2025 Junko Morofuji
#
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

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from crane_x7_description.robot_description_loader import RobotDescriptionLoader

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    description_loader = RobotDescriptionLoader()

    robot_description_semantic_config = load_file(
        'crane_x7_moveit_config', 'config/crane_x7.srdf'
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'crane_x7_moveit_config', 'config/kinematics.yaml'
    )

    # ---------- Launch Arguments ----------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )

    color_arg = DeclareLaunchArgument(
        'color',
        default_value='blue',
        description='cube color (red / blue / yellow)'
    )

    color = LaunchConfiguration('color')

    # ---------- 色ごとの設定 ----------
    color_map = {
        'red': {
            'target_frame': 'red_cube',
            'place': [0.2, -0.2, 0.11],
        },
        'blue': {
            'target_frame': 'blue_cube',
            'place': [0.2, 0.0, 0.11],
        },
        'yellow': {
            'target_frame': 'yellow_cube',
            'place': [0.2, 0.2, 0.11],
        },
    }

    # ---------- Picking Node ----------
    picking_node = Node(
        package='crane_x7_serving_food_examples',
        executable='plate_pick_and_move',
        output='screen',
        parameters=[
            {
                'target_frame': color_map[color.perform(None)]['target_frame'],
                'place_x': color_map[color.perform(None)]['place'][0],
                'place_y': color_map[color.perform(None)]['place'][1],
                'place_z': color_map[color.perform(None)]['place'][2],
                'robot_description': description_loader.load(),
            },
            robot_description_semantic,
            kinematics_yaml,
        ]
    )

    # ---------- Detection Node ----------
    detection_node = Node(
        package='crane_x7_serving_food_examples',
        executable='color_detection',
        output='screen',
        parameters=[
            {'target_color': color}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        color_arg,
        SetParameter(
            name='use_sim_time',
            value=LaunchConfiguration('use_sim_time')
        ),
        detection_node,
        picking_node,
    ])

