from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory('ab_robot'))
    nav = Path(get_package_share_directory('nav2_bringup'))
    gui = LaunchConfiguration('gui')
    auto = LaunchConfiguration('autostart_route')
    report_dir = LaunchConfiguration('report_dir')
    description = xacro.process_file(str(share/'urdf/robot.urdf.xacro')).toxml()
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='simulation_bridge',
                  parameters=[{'use_sim_time':True,'config_file':str(share/'config/bridge.yaml')}])
    sensors=[]
    for name, args, frame in [
        ('lidar_bridge',['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],'lidar_link'),
        ('depth_bridge',['/depth/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                         '/depth@sensor_msgs/msg/Image[ignition.msgs.Image'],'camera_link'),
        ('gps_bridge',['/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'],'gps_link'),
    ]:
        sensors.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name=name,
                            arguments=args,parameters=[{'use_sim_time':True,'override_frame_id':frame}]))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('autostart_route',default_value='true'),
        DeclareLaunchArgument('route_file',default_value=str(share/'config/route.yaml')),
        DeclareLaunchArgument('report_dir',default_value='/ws/artifacts/current'),
        ExecuteProcess(cmd=['ign','gazebo','-r','-s',str(share/'worlds/ab_world.sdf')],output='log'),
        ExecuteProcess(cmd=['ign','gazebo','-g'],condition=IfCondition(gui),output='log'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':description,'use_sim_time':True}]),
        Node(package='ros_gz_sim',executable='create',arguments=['-name','ab_robot','-topic','robot_description',
             '-x','-4','-y','-3','-z','0.0'],parameters=[{'use_sim_time':True}],output='screen'),
        bridge,*sensors,
        Node(package='ab_robot',executable='safety',parameters=[{'use_sim_time':True}]),
        Node(package='ab_robot',executable='audit',parameters=[{'use_sim_time':True,
             'scene_file':str(share/'config/scene.json'),'report_dir':report_dir}]),
        TimerAction(period=4.0,actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav/'launch/bringup_launch.py')),
            launch_arguments={'map':str(share/'maps/ab_map.yaml'),'use_sim_time':'true',
                              'params_file':str(share/'config/nav2.yaml'),
                              'autostart':'true','use_composition':'False'}.items())]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/demo.rviz')],
             parameters=[{'use_sim_time':True}],condition=IfCondition(gui),output='log'),
        Node(package='ab_robot',executable='waypoints',condition=IfCondition(auto),
             parameters=[{'use_sim_time':True,'route_file':LaunchConfiguration('route_file'),
                          'report_file':[report_dir,'/route.json']}],output='screen'),
    ])
