"""Останавливает привод при потере данных или завершении маршрута."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2, NavSatFix
from ros_gz_interfaces.msg import Contacts
from ab_robot.core import command_allowed, obstacle_contact
from std_msgs.msg import String


class Safety(Node):
    def __init__(self):
        super().__init__('sensor_watchdog')
        self.wall_clock = RosClock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter('sensor_timeout_s',5.0)
        self.declare_parameter('command_timeout_s',.5)
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
        self.failed=json.loads(msg.data)['state'] in ('failed','succeeded')

    def contact(self,msg):
        if any(obstacle_contact(c.collision1.name,c.collision2.name) for c in msg.contacts):
            self.collision = True

    def tick(self):
        now=time.monotonic()
        fresh=command_allowed(now,self.times,self.failed or self.collision,
                              self.get_parameter('sensor_timeout_s').value,
                              self.get_parameter('command_timeout_s').value)
        self.pub.publish(self.cmd if fresh else Twist())


def main(args=None):
    rclpy.init(args=args)
    node=Safety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
