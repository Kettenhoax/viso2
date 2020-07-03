from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    root_namespace = '/camera'
    cameras = ['left', 'right']
    components = []
    for camera in cameras:
        sub_nsp = root_namespace + '/' + camera
        components.append(ComposableNode(
            package='image_proc',
            node_plugin='image_proc::DebayerNode',
            node_name='debayer_' + camera,
            node_namespace=sub_nsp,
            # parameters=[
            #     {'camera_namespace': sub_nsp}
            # ],
            remappings=[
                ('image_raw', 'image'),
            ]
        ))
        components.append(ComposableNode(
            package='image_proc',
            node_plugin='image_proc::RectifyNode',
            node_name='rectify_' + camera,
            node_namespace=sub_nsp,
            # parameters=[
            #     {'camera_namespace': sub_nsp, 'image_mono': '/image_mono'}
            # ],
            remappings=[
                ('image', 'image_mono'),
            ]
        ))
    components.append(ComposableNode(
        package='viso2_stereo',
        node_plugin='viso2_stereo::StereoOdometerNode',
        node_name='odometer',
        node_namespace=root_namespace,
        remappings=[
            ('left/image_rect/camera_info', 'left/camera_info'),
            ('right/image_rect/camera_info', 'right/camera_info'),
        ]
    ))

    return LaunchDescription([
        ComposableNodeContainer(
            package='rclcpp_components', node_executable='component_container',
            node_name='image_proc_container', node_namespace=root_namespace,
            composable_node_descriptions=components,
        ),
    ])
