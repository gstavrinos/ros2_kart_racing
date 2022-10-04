#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="kart_description",
            description="The package that includes the robot description.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="kart_robot.urdf.xacro",
            description="The robot description urdf.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "name_suffix",
            default_value="_01",
            description="The robot's name suffix.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "joints_debug",
            default_value="false",
            description="Runs joint state publisher gui if true.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    name_suffix = LaunchConfiguration("name_suffix")
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "team_colours_file",
            default_value=PathJoinSubstitution([FindPackageShare(description_package), "urdf", "include", "colours", "roboskel_team_colours.urdf.xacro"]),
            description="URDF file that includes all the model material colours.",
        )
    )

    #  controller_config = PathJoinSubstitution(
        #  [FindPackageShare("kart_simulation"), "config", "controllers.yaml"]
    #  )

    team_colours_file = LaunchConfiguration("team_colours_file")

    joints_debug = LaunchConfiguration("joints_debug")

    robot_description_content = Command(
         [
             PathJoinSubstitution([FindExecutable(name="xacro")]),
             " ",
             PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
             " ",
             "team_colours_file:=",
             team_colours_file,
             " ",
             "name_suffix:=",
             name_suffix,
             ]
    )

    return LaunchDescription(
        declared_arguments + 
        [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True, "robot_description": robot_description_content, "publish_frequency": 20.0}]
        ),

        #  Node(
            #  package="controller_manager",
            #  executable="spawner",
            #  arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        #  ),

        #  Node(
            #  package="controller_manager",
            #  executable="spawner",
            #  arguments=["kart_ackermann_steering_controller", "-c", "/controller_manager"],
        #  ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
            ),
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_kart",
            arguments=["-entity", "kart", "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0.5"],
            output="screen",
        )
        ],
    )
