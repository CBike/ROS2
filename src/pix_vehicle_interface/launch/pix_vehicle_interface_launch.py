import launch
import launch_ros.actions


def generate_launch_description():
    # Launch pix_interface_cmd_node.py
    cmd_node = launch_ros.actions.Node(
        package='pix_vehicle_interface',
        executable='pix_interface_cmd_node.py',
        name='pix_interface_cmd_node',
        output='screen',
    )

    # Launch pix_interface_rpt_node.py
    rpt_node = launch_ros.actions.Node(
        package='pix_vehicle_interface',
        executable='pix_interface_rpt_node.py',
        name='pix_interface_rpt_node',
        output='screen',
    )

    # Group nodes
    grouped_nodes = launch.actions.GroupAction([
        cmd_node,
        rpt_node
    ])

    return grouped_nodes
