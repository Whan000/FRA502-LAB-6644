from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    eater_turtle_namespace = "eater_turtle"
    killer_turtle_namespace = "killer_turtle"
    sampling_frequency = 20.0

    kill_turtle_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/remove_turtle', 'turtlesim/srv/Kill', '{name: turtle1}'],
        output='screen'
    )

    spawn_eater_turtle_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_turtle', 'turtlesim/srv/Spawn', f'{{x: 5.44, y: 5.44, theta: 0.0, name: {eater_turtle_namespace}}}'],
        output='screen'
    )
    
    spawn_killer_turtle_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_turtle', 'turtlesim/srv/Spawn', f'{{x: 0.44, y: 0.44, theta: 0.0, name: {killer_turtle_namespace}}}'],
        output='screen'
    )

    turtlesim_plus_node = Node(
            package='turtlesim_plus',
            executable='turtlesim_plus_node.py',
            name='turtlesim_plus',
            output='screen'
        )
    
    eater_node = Node(
            package='lab3',
            executable='eater.py',
            name='eater',
            namespace=eater_turtle_namespace,
            parameters=[{
                "sampling_frequency": sampling_frequency
            }],
            output='screen'
        )
    

    
    killer_node = Node(
            package='lab3',
            executable='killer.py',
            name='killer',
            namespace=killer_turtle_namespace,
            parameters=[{
                "sampling_frequency": sampling_frequency,
                "killer_turtle": eater_turtle_namespace
            }],
            output='screen'
        )
     


    ld = LaunchDescription()
    ld.add_action(turtlesim_plus_node)
    ld.add_action(kill_turtle_cmd)
    ld.add_action(spawn_eater_turtle_cmd)
    ld.add_action(spawn_killer_turtle_cmd)
    ld.add_action(eater_node)
    ld.add_action(killer_node)

    return ld