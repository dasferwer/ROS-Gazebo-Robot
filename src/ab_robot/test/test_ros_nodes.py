from pathlib import Path
import pytest
rclpy = pytest.importorskip('rclpy')
from ab_robot.waypoints import Waypoints
from ab_robot.safety import Safety


def test_nodes_construct_with_real_ros_context():
    route = str(Path(__file__).resolve().parents[1]/'config/route.yaml')
    rclpy.init(args=['--ros-args','-p','route_file:='+route])
    nodes=[]
    try:
        nodes.append(Safety())
        nodes.append(Waypoints())
        assert nodes[-1].report['state']=='waiting'
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


def test_startup_timeout_is_terminal(tmp_path):
    import time
    import yaml
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/route.yaml').read_text())
    cfg['startup_timeout_s']=.01
    route=tmp_path/'route.yaml'
    route.write_text(yaml.safe_dump(cfg))
    report=tmp_path/'report.json'
    rclpy.init(args=['--ros-args','-p','route_file:='+str(route),'-p','report_file:='+str(report),'-p','use_sim_time:=true'])
    node=Waypoints()
    try:
        deadline=time.monotonic()+2
        while not node.done and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        assert node.done and not node.report['success']
        assert 'Тайм-аут' in node.report['reason']
        assert report.exists()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
