#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, LocalSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
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

    team_colours_file = LaunchConfiguration("team_colours_file")

    joints_debug = LaunchConfiguration("joints_debug")
    jsp_gui = "" if not LocalSubstitution("joints_debug") else "_gui"

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
        # TODO use name_suffix in name instead of the hardcoded mess
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher"+"_01",
            output="screen",
            parameters=[{"use_sim_time": True, "robot_description": robot_description_content, "publish_frequency": 20.0, "ignore_timestamp": False,
            "frame_prefix":"",}],
            remappings=[("robot_description","robot_description"+"_01")]
        ),

        #  Node(
            #  package="joint_state_publisher"+jsp_gui,
            #  executable="joint_state_publisher"+jsp_gui,
            #  name="joint_state_publisher"+jsp_gui,
            #  output="screen")
        ]
    )
