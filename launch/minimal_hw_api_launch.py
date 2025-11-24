from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='hw_api_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='mrs_multirotor_simulator',
                    plugin='mrs_uav_fcu_api::MrsUavFcuApi',
                    name='hw_api'
                )
            ],
            output='screen'
        )
    ])