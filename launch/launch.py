from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from project4c.disc_robot import load_disc_robot




def generate_launch_description():

    bag_out_arg = DeclareLaunchArgument('bag_out') ## argument to enter at the terminal for the output bag file name
    # Using Command substitution to concatenate path and argument
    bag_record_file_path = Command(["echo /home/lab2004/Fall_2023/CSCE_752/project4c/recordings/", LaunchConfiguration('bag_out')])  # Using Command substitution to concatenate path and argument
 
    robot_file = 'normal.robot'
    world_file = 'donut.world'  ##can be changed according to the desired world


    ep2 = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','--all','-o', bag_record_file_path],
        shell=True,  # Added this to ensure that the command is executed in the shell
        name='ros2_bag_record'  # Given a name to the process for identification
        )

    robot_name_file_path= f'/home/lab2004/project_ws/src/project4a/project4a/{robot_file}'
    world_name_file_path= f'/home/lab2004/project_ws/src/project4c/project4c/{world_file}'


    robot = load_disc_robot(robot_name_file_path)
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot['urdf']}],
        )
    velocity_translator_node = Node(
            package='project4c',
            executable='velocity_translator',
            name='velocity_translator',
            output='screen',
            parameters=[{'robot_name':robot_name_file_path}]
        )
    
    differential_drive_simulator_node = Node(
            package='project4c',
            executable='differential_drive_simulator',
            name='differential_drive_simulator',
            output='screen',
            parameters=[{'robot_name':robot_name_file_path},{'world_name':world_name_file_path}],
        )
    
    navigation_controller_node = Node(
            package='project4c',
            executable='navigation_controller_2',
            name='navigation_controller_2',
            output='screen',
        )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        )
    return LaunchDescription([
        bag_out_arg,
        robot_state_publisher_node,
        velocity_translator_node,
        differential_drive_simulator_node,
        navigation_controller_node,
        rviz_node,
        ep2
        

    ])