#!/usr/bin/env python3
import os
import math
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    competition_mode = LaunchConfiguration("competition_mode")

    competition_mode_launch_arg = DeclareLaunchArgument(
        "competition_mode",
        default_value="false"
    )

    track_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("race_tracks"), "/launch", "/kartland.launch.py"]
        ),
    )

    kart_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("kart_description"), "/launch", "/description.launch.py"]
        ),
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("kart_navigation"), "/launch", "/navigation.launch.py"]
        ),
    )

    # TODO Fix the hardcoded _01 ugliness
    # TODO adding the suffix here does not load the gazebo ros2_control plugin properly
    kart_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_kart",
        arguments=["-entity", "kart"+"_01", "-topic", "robot_description"+"_01", "-x", "6.7357", "-y", "-9.9246", "-z", "0.4", "-Y", str(math.pi/7)],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"+"_01"],
        output="screen"
    )

    load_joint_position_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_position_controller"+"_01"],
        output="screen"
    )

    load_velocity_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start",
             "velocity_controller"+"_01"],
        output="screen"
    )

    unpause_physics = ExecuteProcess(
        condition=IfCondition(competition_mode),
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/unpause_physics ',
            'std_srvs/Empty '
            '{}'
            ]],
        shell=True
    )

    start_racing = ExecuteProcess(
        condition=IfCondition(competition_mode),
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic pub ',
            '/race_steward/go ',
            'std_msgs/Empty ',
            '{} --once'
            ]],
        shell=True
    )

    judge_go = ExecuteProcess(
        condition=IfCondition(competition_mode),
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic pub ',
            '/race_steward/judge_go ',
            'std_msgs/Empty ',
            '{} --once'
            ]],
        shell=True
    )

    return LaunchDescription(
        [
        competition_mode_launch_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
                ),
            launch_arguments={"world": os.path.join(FindPackageShare("race_tracks").find("race_tracks"), "worlds", "race_track.world"), "pause":"true"}.items()
        ),

        # NOTE
        # Not separating the nodes in group actions
        # results in variable conflicts
        # Until this is fixed (known ros2 launch bug)
        # This is the easiest fix I could find
        # (Most probably must be applied to multiple robots too)
        GroupAction(
            actions=[
                kart_description_launch,
            ]
        ),

        GroupAction(
            actions=[
                track_description_launch,
            ]
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_track",
            arguments=["-entity", "kartland", "-topic", "track_description", "-x", "0", "-y", "0", "-z", "0.16"],
            output="screen",
        ),

        kart_spawn,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=kart_spawn,
                on_exit=[load_joint_state_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_position_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=kart_spawn,
                on_exit=[
                    TimerAction(
                        period = 5.0,
                        actions = [unpause_physics]
                    )
                ],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=unpause_physics,
                on_exit=[
                    TimerAction(
                        period = 1.0,
                        actions = [judge_go]
                    )
                ],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=judge_go,
                on_start=[
                    TimerAction(
                        period = 2.0,
                        actions = [
                            GroupAction(
                                actions=[navigation_launch]
                            ),
                        ]
                    )
                ],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=judge_go,
                on_start=[
                    TimerAction(
                        period = 20.0,
                        actions = [start_racing]
                    )
                ],
            )
        ),

        ],
    )

