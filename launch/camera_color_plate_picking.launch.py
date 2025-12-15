from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # 引数（launch時に切り替える）
    detect_color = LaunchConfiguration('detect_color')
    target_frame = LaunchConfiguration('target_frame')

    return LaunchDescription([

        # --- 引数定義 ---
        DeclareLaunchArgument(
            'detect_color',
            default_value='blue',
            description='Color to detect: blue / yellow / red'
        ),

        DeclareLaunchArgument(
            'target_frame',
            default_value='blue_cube',
            description='TF frame name of target object'
        ),

        # --- 色検出ノード ---
        Node(
            package='crane_x7_serving_food_examples',
            executable='color_detection',
            name='color_detection',
            output='screen',
            parameters=[{
                'detect_color': detect_color
            }]
        ),

        # --- Pick & Place ノード ---
        Node(
            package='crane_x7_serving_food_examples',
            executable='plate_pick_and_move_tf',
            name='pick_and_place',
            output='screen',
            parameters=[{
                'target_frame': target_frame,
                'place_x': 0.2,
                'place_y': 0.2,
                'place_z': 0.1
            }]
        ),
    ])

