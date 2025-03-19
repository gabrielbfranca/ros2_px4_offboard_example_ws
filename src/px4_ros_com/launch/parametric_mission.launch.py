from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',  # Replace with your package name
            executable='parametric_mission.py',  # This should match your script's entry point
            name='parametric_mission',
            output='screen',
            parameters=[
                {'altitude': -10.0},
                {'velocity': 3.0},
                {'destination': [20.0, 15.0]},
                {'position_tolerance': 5.0},
                {'max_acceleration': 2.5},
                {'max_yaw_angle': 45.0},
                {'climb_rate': 1.5},
                {'descent_rate': 1.5},

            ]
        )
    ])
"""
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',  # Your package name
            executable='parametric_mission.py',  # Your script name
            name='parametric_mission',
            output='screen',
            parameters=[
                {'altitude': -10.0},
                {'velocity': 3.0},
                {'destination': [20.0, 15.0]},
                {'position_tolerance': 0.5}
            ]
        )
    ])
"""
