#青→黄→赤と連続にものを掴む
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # ===== 青 =====
    blue_detection = Node(
        package='crane_x7_serving_food_examples',
        executable='color_detection',
        name='color_detection_blue',
        output='screen',
        parameters=[{'detect_color': 'blue'}]
    )

    blue_pick = Node(
        package='crane_x7_serving_food_examples',
        executable='plate_pick_and_move_tf',
        name='pick_blue',
        output='screen',
        parameters=[{
            'target_frame': 'blue_cube',
            'place_x': 0.2,
            'place_y': 0.0,
            'place_z': 0.1
        }]
    )

    # ===== 黄 =====
    yellow_detection = Node(
        package='crane_x7_serving_food_examples',
        executable='color_detection',
        name='color_detection_yellow',
        output='screen',
        parameters=[{'detect_color': 'yellow'}]
    )

    yellow_pick = Node(
        package='crane_x7_serving_food_examples',
        executable='plate_pick_and_move_tf',
        name='pick_yellow',
        output='screen',
        parameters=[{
            'target_frame': 'yellow_cube',
            'place_x': 0.2,
            'place_y': 0.2,
            'place_z': 0.1
        }]
    )

    # ===== 赤 =====
    red_detection = Node(
        package='crane_x7_serving_food_examples',
        executable='color_detection',
        name='color_detection_red',
        output='screen',
        parameters=[{'detect_color': 'red'}]
    )

    red_pick = Node(
        package='crane_x7_serving_food_examples',
        executable='plate_pick_and_move_tf',
        name='pick_red',
        output='screen',
        parameters=[{
            'target_frame': 'red_cube',
            'place_x': 0.2,
            'place_y': -0.2,
            'place_z': 0.1
        }]
    )

    return LaunchDescription([

        # 青
        blue_detection,
        blue_pick,

        # 黄（２０秒後）
        TimerAction(
            period=10.0,
            actions=[yellow_detection, yellow_pick]
        ),

        # 赤（４０秒後）
        TimerAction(
            period=20.0,
            actions=[red_detection, red_pick]
        ),
    ])

