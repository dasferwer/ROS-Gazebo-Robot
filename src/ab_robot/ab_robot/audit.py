"""Сохраняет живые показания и проверку геометрии независимо от Nav2."""
from collections import Counter
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, NavSatFix, JointState
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
from ab_robot.core import obstacle_contact


def rect_separation(x, y, yaw, obstacle):
    # SAT даёт знак пересечения прямоугольников, без приближения кругом.
    c,s=math.cos(yaw),math.sin(yaw)
    robot=[(x+c*a-s*b,y+s*a+c*b) for a,b in [(-.23,-.23),(-.23,.23),(.26,.23),(.26,-.23)]]
    ox,oy,hx,hy=obstacle['x'],obstacle['y'],obstacle['sx']/2,obstacle['sy']/2
    other=[(ox+a,oy+b) for a,b in [(-hx,-hy),(-hx,hy),(hx,hy),(hx,-hy)]]
    gaps=[]
    for ax,ay in [(1,0),(0,1),(c,s),(-s,c)]:
        rp=[px*ax+py*ay for px,py in robot]
        op=[px*ax+py*ay for px,py in other]
        gaps.append(max(min(op)-max(rp),min(rp)-max(op)))
    return max(gaps)


class Audit(Node):
    def __init__(self):
        super().__init__('simulation_audit')
        self.wall_clock = RosClock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter('scene_file','')
        self.declare_parameter('report_dir','/ws/artifacts/current')
        self.scene=json.loads(Path(self.get_parameter('scene_file').value).read_text())
        self.out=Path(self.get_parameter('report_dir').value)
        self.out.mkdir(parents=True,exist_ok=True)
        self.counts=Counter()
        self.frames={}
        self.tf_sources={}
        self.tf_stamps={}
        self.sensor_stamps={}
        self.trajectory=[]
        self.truth=None
        self.contacts=[]
        self.route={}
        self.last_command=None
        self.controller_trace={}
        self.navigation_trace=[]
        self.create_subscription(NavPath,'/local_plan',self.local_plan,10)
        self.create_subscription(PolygonStamped,'/local_costmap/published_footprint',self.footprint,10)
        self.create_subscription(PolygonStamped,'/global_costmap/published_footprint',self.global_footprint,10)
        self.create_subscription(Twist,'/cmd_vel_safe',self.command,10)
        self.min_gap=math.inf
        self.overlap_samples=0
        self.depth_low_marked=0
        self.finite_depth=0
        self.last_clock=None
        self.clock_regressions=0
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        for topic,typ in [('/scan',LaserScan),('/depth/points',PointCloud2),('/gps/fix',NavSatFix),
                          ('/odom',Odometry),('/joint_states',JointState)]:
            self.create_subscription(typ,topic,lambda m,t=topic:self.sensor(t,m),qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/ground_truth',self.ground_truth,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf',self.tf,qos_profile_sensor_data)
        for topic in ['/contacts/body','/contacts/left','/contacts/right']:
            self.create_subscription(Contacts,topic,lambda m,t=topic:self.contact(t,m),qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.costmap,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/route_status',self.status,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.2,self.sample,clock=self.wall_clock)
        self.create_timer(2.0,self.save,clock=self.wall_clock)

    def command(self,msg):
        self.counts['/cmd_vel_safe']+=1
        self.last_command=[msg.linear.x,msg.angular.z]

    def local_plan(self,msg):
        self.controller_trace['local_plan_stamp']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.controller_trace['local_plan_frame']=msg.header.frame_id
        if msg.poses:
            p=msg.poses[0].pose.position
            self.controller_trace['local_plan_start']=[p.x,p.y]

    def footprint(self,msg):
        self.controller_trace['footprint_stamp']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.controller_trace['footprint_frame']=msg.header.frame_id
        if msg.polygon.points:
            self.controller_trace['footprint_center']=[sum(getattr(p,k) for p in msg.polygon.points)/len(msg.polygon.points) for k in ('x','y')]

    def global_footprint(self,msg):
        self.controller_trace['global_footprint_stamp']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if msg.polygon.points:
            self.controller_trace['global_footprint_center']=[sum(getattr(p,k) for p in msg.polygon.points)/len(msg.polygon.points) for k in ('x','y')]

    def sensor(self,topic,msg):
        self.counts[topic]+=1
        self.frames[topic]=msg.header.frame_id
        self.sensor_stamps[topic]=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if topic=='/depth/points':
            offsets={f.name:f.offset for f in msg.fields}
            if 'z' in offsets:
                endian='>' if msg.is_bigendian else '<'
                self.finite_depth=sum(math.isfinite(struct.unpack_from(endian+'f',msg.data,i+offsets['z'])[0])
                                      for i in range(0,len(msg.data),msg.point_step*8))

    def clock(self,msg):
        self.counts['/clock']+=1
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.last_clock is not None and value<self.last_clock:
            self.clock_regressions+=1
        self.last_clock=value

    def tf(self,msg):
        for tf in msg.transforms:
            self.tf_sources.setdefault(tf.child_frame_id,set()).add(tf.header.frame_id)
            self.tf_stamps[tf.child_frame_id]=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9

    def ground_truth(self,msg):
        self.counts['/ground_truth']+=1
        for tf in msg.transforms:
            if tf.child_frame_id=='ab_robot':
                self.truth=tf.transform

    def contact(self,topic,msg):
        self.counts[topic]+=1
        for contact in msg.contacts:
            names=[contact.collision1.name,contact.collision2.name]
            if obstacle_contact(*names):
                if len(self.contacts)<100:
                    self.contacts.append({'topic':topic,'collisions':names,'time':self.last_clock})

    def costmap(self,msg):
        self.counts['/global_costmap/costmap']+=1
        low=next(o for o in self.scene['obstacles'] if o['name']=='low')
        col=int((low['x']-msg.info.origin.position.x)/msg.info.resolution)
        row=int((low['y']-msg.info.origin.position.y)/msg.info.resolution)
        if 0<=col<msg.info.width and 0<=row<msg.info.height:
            index=row*msg.info.width+col
            if msg.data[index]>=99:
                self.depth_low_marked+=1

    def status(self,msg):
        self.route=json.loads(msg.data)
        self.save()

    def sample(self):
        if self.truth is None:
            return
        t=self.truth.translation
        q=self.truth.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        gaps=[rect_separation(t.x,t.y,yaw,o) for o in self.scene['obstacles']]
        gap=min(gaps)
        self.min_gap=min(self.min_gap,gap)
        self.overlap_samples+=int(gap<=0)
        self.trajectory.append([self.last_clock,t.x,t.y,yaw,gap])
        self.navigation_trace.append({'sim':self.last_clock,'truth':[t.x,t.y],
                                     'tf_stamps':dict(self.tf_stamps),**self.controller_trace})

    def save(self):
        transforms={}
        for target in ['base_link','lidar_link','camera_optical_frame','gps_link']:
            try:
                tf=self.buffer.lookup_transform('map',target,rclpy.time.Time())
                transforms[target]=[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
            except TransformException:
                transforms[target]=None
        finish=self.scene['finish']
        error=None if self.truth is None else math.hypot(self.truth.translation.x-finish[0],self.truth.translation.y-finish[1])
        data={'messages':dict(self.counts),'frames':self.frames,'map_transforms':transforms,
              'tf_stamps':self.tf_stamps,'sensor_stamps':self.sensor_stamps,
              'tf_parents':{k:sorted(v) for k,v in self.tf_sources.items()},
              'clock_regressions':self.clock_regressions,'sim_seconds':self.last_clock,
              'finite_depth_samples':self.finite_depth,'depth_low_obstacle_marked_updates':self.depth_low_marked,
              'obstacle_contacts':self.contacts,'geometry_overlap_samples':self.overlap_samples,
              'minimum_sat_separation_m':None if math.isinf(self.min_gap) else self.min_gap,
              'last_safe_command':self.last_command,'ground_truth_finish_error_m':error,'trajectory_samples':len(self.trajectory),'route':self.route}
        (self.out/'audit.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
        (self.out/'trajectory.json').write_text(json.dumps(self.trajectory)+'\n')
        (self.out/'navigation_trace.json').write_text(json.dumps(self.navigation_trace)+'\n')


def main(args=None):
    rclpy.init(args=args)
    node=Audit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()
