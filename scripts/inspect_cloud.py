import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
rclpy.init()
n=Node('inspect_cloud')
def cb(m):
 print(m.header.frame_id, [(f.name,f.offset) for f in m.fields], m.width,m.height)
 for index in [0,len(m.data)//m.point_step//2, len(m.data)//m.point_step//2+m.width//2]:
  print(index,struct.unpack_from('<fff',m.data,index*m.point_step))
 n.done=True
n.done=False
s=n.create_subscription(PointCloud2,'/depth/points',cb,qos_profile_sensor_data)
while not n.done: rclpy.spin_once(n,timeout_sec=1)
n.destroy_node();rclpy.shutdown()
