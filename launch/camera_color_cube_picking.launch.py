from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    blue_detect = Node(
        package='crane_x7_serving_food_examples',
        executable='color_detection',
        name='color_detection_blue',
        output='screen',
        parameters=[{
            'detect_color': 'blue'
        }]
    )

    blue_pick = Node(
        package='crane_x7_serving_food_examples',
        executable='plate_pick_and_move_tf',
        name='pick_and_place_blue',
        output='screen',
        parameters=[{
            'target_frame': 'blue_cube',
            'place_x': 0.2,
            'place_y': 0.2,
            'place_z': 0.1
        }]
    )

    yellow_group = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='crane_x7_serving_food_examples',
                executable='color_detection',
                name='color_detection_yellow',
                output='screen',
                parameters=[{
                    'detect_color': 'yellow'
                }]
            ),
            Node(
                package='crane_x7_serving_food_examples',
                executable='plate_pick_and_move_tf',
                name='pick_and_place_yellow',
                output='screen',
                parameters=[{
                    'target_frame': 'yellow_cube',
                    'place_x': 0.0,
                    'place_y': 0.2,
                    'place_z': 0.1
                }]
            )
        ]
    )

    red_group = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='crane_x7_serving_food_examples',
                executable='color_detection',
                name='color_detection_red',
                output='screen',
                parameters=[{
                    'detect_color': 'red'
                }]
            ),
            Node(
                package='crane_x7_serving_food_examples',
                executable='plate_pick_and_move_tf',
                name='pick_and_place_red',
                output='screen',
                parameters=[{
                    'target_frame': 'red_cube',
                    'place_x':_

