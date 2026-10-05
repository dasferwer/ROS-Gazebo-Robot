"""Останавливает привод при потере данных или завершении маршрута."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PolygonStamped
from sensor_msgs.msg import LaserScan, PointCloud2, NavSatFix
from ros_gz_interfaces.msg import Contacts
from ab_robot.core import command_allowed, obstacle_contact
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class Safety(Node):
    def __init__(self):
        super().__init__('sensor_watchdog')
        self.wall_clock = RosClock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter('sensor_timeout_s',5.0)
        self.declare_parameter('command_timeout_s',.5)
        self.declare_parameter('tf_timeout_s',.5)
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.last_tf_stamp=None
        self.tf_progress=0.0
        self.declare_parameter('controller_pose_timeout_s',.5)
        self.declare_parameter('controller_pose_tolerance_m',.25)
        self.route_running=False
        self.localization_fault=False
        self.controller_pose=None
        self.controller_received=0.0
        self.controller_bad_since=None
        self.fault_pub=self.create_publisher(String,'/safety_fault',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PolygonStamped,'/local_costmap/published_footprint',self.footprint,10)
        self.collision = False
        self.cmd = Twist()
        self.times = {}
        self.failed = False
        self.pub = self.create_publisher(Twist,'/cmd_vel_safe',10)
        self.create_subscription(Twist,'/cmd_vel',self.command,10)
        self.create_subscription(LaserScan,'/scan',lambda m:self.mark('scan'),qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/depth/points',lambda m:self.mark('depth'),qos_profile_sensor_data)
        self.create_subscription(NavSatFix,'/gps/fix',lambda m:self.mark('gps'),qos_profile_sensor_data)
        for topic in ('/contacts/body','/contacts/left','/contacts/right'):
            self.create_subscription(Contacts,topic,self.contact,qos_profile_sensor_data)
        self.create_subscription(String,'/route_status',self.status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05,self.tick,clock=self.wall_clock)

    def mark(self,name):
        self.times[name]=time.monotonic()

    def command(self,msg):
        self.cmd=msg
        self.mark('cmd')

    def status(self,msg):
        state=json.loads(msg.data)['state']
        self.failed=self.localization_fault or state in ('failed','succeeded')
        self.route_running=state=='running'

    def footprint(self,msg):
        if msg.header.frame_id=='map' and msg.polygon.points:
            self.controller_pose=(msg.header.stamp,
                sum(p.x for p in msg.polygon.points)/len(msg.polygon.points),
                sum(p.y for p in msg.polygon.points)/len(msg.polygon.points))
            self.controller_received=time.monotonic()

    def contact(self,msg):
        if any(obstacle_contact(c.collision1.name,c.collision2.name) for c in msg.contacts):
            self.collision = True

    def tick(self):
        now=time.monotonic()
        fresh=command_allowed(now,self.times,self.failed or self.collision,
                              self.get_parameter('sensor_timeout_s').value,
                              self.get_parameter('command_timeout_s').value)
        try:
            tf=self.buffer.lookup_transform('map','base_link',rclpy.time.Time())
            stamp=rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds
            if stamp!=self.last_tf_stamp:
                self.last_tf_stamp=stamp
                self.tf_progress=now
            timeout=self.get_parameter('tf_timeout_s').value
            age=(self.get_clock().now().nanoseconds-stamp)*1e-9
            fresh=fresh and -.5<=age<timeout and now-self.tf_progress<timeout
            if self.route_running:
                controller_ok=False
                if self.controller_pose:
                    header,x,y=self.controller_pose
                    pose_age=(self.get_clock().now().nanoseconds-rclpy.time.Time.from_msg(header).nanoseconds)*1e-9
                    distance=((x-tf.transform.translation.x)**2+(y-tf.transform.translation.y)**2)**.5
                    limit=self.get_parameter('controller_pose_timeout_s').value
                    controller_ok=(-.5<=pose_age<limit and now-self.controller_received<limit
                                   and distance<self.get_parameter('controller_pose_tolerance_m').value)
                fresh=fresh and controller_ok
                if controller_ok:
                    self.controller_bad_since=None
                elif self.controller_bad_since is None:
                    self.controller_bad_since=now
                elif now-self.controller_bad_since>.3:
                    self.failed=True
                    self.localization_fault=True
                    self.route_running=False
                    self.fault_pub.publish(String(data='Поза local costmap устарела или расходится с TF; привод остановлен'))
        except TransformException:
            fresh=False
        self.pub.publish(self.cmd if fresh else Twist())


def main(args=None):
    rclpy.init(args=args)
    node=Safety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
