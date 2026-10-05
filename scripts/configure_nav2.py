#!/usr/bin/env python3
from pathlib import Path
import yaml

root = Path(__file__).resolve().parents[1]
# Исходная конфигурация Nav2 Humble, Apache-2.0; ссылка в README.
p = yaml.safe_load((root/'docs/nav2_humble_defaults.yaml').read_text())
a = p['amcl']['ros__parameters']
a.update(base_frame_id='base_footprint',set_initial_pose=True,
         initial_pose={'x':-4.0,'y':-3.0,'z':0.0,'yaw':0.0},
         min_particles=300,max_particles=800,update_min_d=.08,update_min_a=.1,
         laser_max_range=8.0,transform_tolerance=.5,alpha1=.03,alpha2=.03,alpha3=.03,alpha4=.03)
c=p['controller_server']['ros__parameters']
c['controller_frequency']=10.0
c['progress_checker'].update(required_movement_radius=.15,movement_time_allowance=15.0)
c['general_goal_checker'].update(xy_goal_tolerance=.18,yaw_goal_tolerance=.2)
c['FollowPath'].update(max_vel_x=.35,max_speed_xy=.35,max_vel_theta=.8,
                      acc_lim_x=.7,decel_lim_x=-.7,acc_lim_theta=1.5,decel_lim_theta=-1.5,
                      vx_samples=12,vy_samples=1,vtheta_samples=16,sim_time=1.5,
                      trans_stopped_velocity=.05,xy_goal_tolerance=.18,
                      debug_trajectory_details=False,transform_tolerance=.5)
c['FollowPath']['critics']=['RotateToGoal','Oscillation','ObstacleFootprint','GoalAlign','PathAlign','PathDist','GoalDist']
c['FollowPath'].pop('BaseObstacle.scale',None)
c['FollowPath']['ObstacleFootprint.scale']=1.0
footprint='[[-0.23, -0.23], [-0.23, 0.23], [0.26, 0.23], [0.26, -0.23]]'
for name in ['local_costmap','global_costmap']:
    c=p[name][name]['ros__parameters']
    c.pop('robot_radius',None)
    c.pop('voxel_layer',None)
    c.update(footprint=footprint,footprint_padding=.02,transform_tolerance=.5,
             update_frequency=5.0,publish_frequency=2.0,
             plugins=['static_layer','lidar_layer','depth_layer','inflation_layer'] if name=='global_costmap' else ['lidar_layer','depth_layer','inflation_layer'])
    if name=='local_costmap': c.update(width=5,height=5)
    c['inflation_layer'].update(inflation_radius=.7,cost_scaling_factor=3.0)
    c['obstacle_layer']={
        'plugin':'nav2_costmap_2d::ObstacleLayer', 'enabled':True,
        'observation_sources':'scan depth',
        'scan':dict(topic='/scan',data_type='LaserScan',sensor_frame='lidar_link',
                    marking=True,clearing=True,max_obstacle_height=1.5,
                    obstacle_max_range=7.5,raytrace_max_range=8.0,obstacle_min_range=.12,
                    raytrace_min_range=.12,inf_is_valid=True),
        'depth':dict(topic='/depth/points',data_type='PointCloud2',sensor_frame='camera_link',
                     marking=True,clearing=True,min_obstacle_height=.06,max_obstacle_height=1.5,
                     obstacle_max_range=4.8,raytrace_max_range=5.0,obstacle_min_range=.2,
                     raytrace_min_range=.2),
    }
    sources=c.pop('obstacle_layer')
    c['lidar_layer']={'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True,
                      'observation_sources':'scan','scan':sources['scan']}
    c['depth_layer']={'plugin':'nav2_costmap_2d::VoxelLayer','enabled':True,
                      'observation_sources':'depth','depth':sources['depth'],
                      'origin_z':0.0,'z_resolution':.05,'z_voxels':16,
                      'mark_threshold':0,'max_obstacle_height':.8,
                      'publish_voxel_map':True}
v=p['velocity_smoother']['ros__parameters']
v.update(max_velocity=[.35,0.0,.8],min_velocity=[0.0,0.0,-.8],
         max_accel=[.7,0.0,1.5],max_decel=[-.7,0.0,-1.5],velocity_timeout=.5)
(root/'src/ab_robot/config/nav2.yaml').write_text(yaml.safe_dump(p,sort_keys=False))
