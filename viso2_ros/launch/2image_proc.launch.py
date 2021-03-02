from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    root_namespace = '/usb_cam'
    components = []
    #components.append(ComposableNode(
    #        package='image_proc',
   #         node_plugin='image_proc::DebayerNode',
  #          node_name='debayer_camera',
 #           node_namespace=root_namespace,
            # parameters=[
            #     {'camera_namespace': root_namespace}
            # ],
            #remappings=[
            #    ('image_raw', 'image/compressed'),
            #]
   #     ))
  #  components.append(ComposableNode(
    #        package='image_proc',
   #         node_plugin='image_proc::RectifyNode',
  #          node_name='rectify_camera',
 #           node_namespace=root_namespace,
         #   # parameters=[
        #    #     {'camera_namespace': root_namespace, 'image_mono': '/image_mono'}
       #     # ],
      #      remappings=[
     #           ('image', 'image_mono'),
    #        ]
   #     ))
    components.append(ComposableNode(
        package='viso2_ros',
        node_plugin='viso2_ros::MonoOdometerNode',
        node_name='odometer',
        node_namespace=root_namespace,
        remappings=[
            ('image', 'image_raw'),
        ]
    ))

    return LaunchDescription([
        ComposableNodeContainer(
            package='rclcpp_components', node_executable='component_container',
            node_name='image_proc_container', node_namespace=root_namespace,
            composable_node_descriptions=components,
        ),
    ])
