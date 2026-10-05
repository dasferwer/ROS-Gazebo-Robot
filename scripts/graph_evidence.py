#!/usr/bin/env python3
import json
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage

rclpy.init()
n=Node('graph_evidence')
parents={}
sources={}

import subprocess
root=Path(__file__).resolve().parents[1]
build=root/'artifacts/tf_evidence-build'
with (root/'artifacts/tf-evidence-build.log').open('w') as log:
    subprocess.run(['cmake','-S',str(root/'scripts/tf_evidence'),'-B',str(build)],check=True,stdout=log,stderr=log)
    subprocess.run(['cmake','--build',str(build),'-j','2'],check=True,stdout=log,stderr=log)
output=subprocess.check_output([str(build/'tf_evidence')],text=True)
for line in output.splitlines():
    parent,child,gid=line.split()
    sources.setdefault(parent+' -> '+child,set()).add(gid)
endpoints={}
for topic in ['/scan','/depth/points','/gps/fix','/clock','/tf','/tf_static']:
    endpoints[topic]={kind:[{'node':e.node_name,'gid':bytes(e.endpoint_gid).hex()} for e in method(topic)]
                      for kind,method in [('publishers',n.get_publishers_info_by_topic),('subscribers',n.get_subscriptions_info_by_topic)]}
data={'endpoints':endpoints,'tf_edge_publisher_gids':{k:sorted(v) for k,v in sources.items()},
      'nodes':[ns+'/'+name for name,ns in n.get_node_names_and_namespaces()]}
path=Path(sys.argv[1]);path.write_text(json.dumps(data,indent=2)+'\n')
print('Лидар и depth имеют потребителей:',[e['node'] for e in endpoints['/depth/points']['subscribers']])
print('Издатели каждого ребра TF:', {k:len(v) for k,v in sources.items()})
n.destroy_node();rclpy.shutdown()
