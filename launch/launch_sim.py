import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='fejemis_sim' 

    model_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','launch_model_sim.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()   
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )


    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_params_path = os.path.join(
                    get_package_share_directory(package_name),'config','gazebo_params.yaml')
    
    world_file_path = 'fejemis_sim/worlds/sim_world' #your path to the world file
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra gazebo args': '--ros-args --params-file ' + gazebo_params_path, 'world': world_file_path}.items(),
            )
    

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'fejemis'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    rviz2 = Node(
        package="rviz2",
        namespace='',
        executable="rviz2",
        name="rviz2",
        arguments=['-d' + os.path.join(get_package_share_directory('fejemis_sim'), 'config', 'main.rviz')]
    )

    # slam = Node(
    #     package="slam_toolbox",
    #     namespace='',
    #     executable="online_async_launch.py",
    #     name="rviz2",
    #     arguments=['params_file:=./fejemis_sim/config/mapper_params_online_async.yaml use_sim_time:=true']
    # )

    # twist_mux = Node(
    #     package="twist_mux",
    #     executable="twist_mux",
    #     arguments=['--params-file' + os.path.join(get_package_share_directory('fejemis_sim'), 'config', 'twist_mux.yaml'), '-r' + 'cmd_vel_out:=diff_cont/cmd_vel_unstamped']
    # )




    # Launch them all!
    return LaunchDescription([
        model_sim,
        joystick,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        rviz2,
    ])
