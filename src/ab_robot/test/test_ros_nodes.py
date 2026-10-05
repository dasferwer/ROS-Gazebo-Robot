from pathlib import Path
import pytest
rclpy = pytest.importorskip('rclpy')
from ab_robot.waypoints import Waypoints
from ab_robot.safety import Safety


def test_watchdog_blocks_commands_when_localization_tf_is_stale():
    from geometry_msgs.msg import TransformStamped, Twist
    from rclpy.time import Time
    import time
    rclpy.init()
    node=Safety()
    output=[]
    node.pub=type('Publisher',(),{'publish':lambda self,msg:output.append(msg.linear.x)})()
    try:
        tf=TransformStamped()
        tf.header.frame_id='map'
        tf.child_frame_id='base_link'
        tf.transform.rotation.w=1.0
        tf.header.stamp=Time(nanoseconds=node.get_clock().now().nanoseconds-2_000_000_000).to_msg()
        node.buffer.set_transform(tf,'test')
        node.times={k:time.monotonic() for k in ('scan','depth','gps','cmd')}
        node.cmd=Twist();node.cmd.linear.x=.3
        node.tick()
        assert output==[0.0]
        tf.header.stamp=node.get_clock().now().to_msg()
        node.buffer.set_transform(tf,'test')
        node.tick()
        assert output[-1]==.3
        node.tf_progress=time.monotonic()-2
        node.tick()
        assert output[-1]==0.0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


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


@pytest.mark.parametrize('already_done',[False,True])
def test_terminal_state_cancels_goal_acknowledgement(tmp_path,already_done):
    from rclpy.task import Future
    route=str(Path(__file__).resolve().parents[1]/'config/route.yaml')
    rclpy.init(args=['--ros-args','-p','route_file:='+route,'-p','report_file:='+str(tmp_path/'report.json')])
    node=Waypoints()
    cancelled=[]
    class Handle:
        accepted=True
        def cancel_goal_async(self):
            cancelled.append(True)
    future=Future()
    node.send_future=future
    if already_done:
        future.set_result(Handle())
    try:
        node.finish(False,'Тайм-аут цели')
        if not already_done:
            future.set_result(Handle())
        assert cancelled==[True]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_shutdown_preserves_failure_report(tmp_path):
    route=str(Path(__file__).resolve().parents[1]/'config/route.yaml')
    report=tmp_path/'shutdown.json'
    rclpy.init(args=['--ros-args','-p','route_file:='+route,'-p','report_file:='+str(report)])
    node=Waypoints()
    rclpy.shutdown()
    try:
        node.finish(False,'Остановлено пользователем')
        assert report.exists()
    finally:
        node.destroy_node()


def test_waypoints_cancel_active_mission_on_missing_tf(tmp_path):
    import time
    from ab_robot.core import GeoReference
    route=str(Path(__file__).resolve().parents[1]/'config/route.yaml')
    rclpy.init(args=['--ros-args','-p','route_file:='+route,'-p','report_file:='+str(tmp_path/'tf.json')])
    node=Waypoints()
    cancelled=[]
    class Handle:
        def cancel_goal_async(self):
            cancelled.append(True)
    try:
        node.reference=GeoReference(55.75,37.61)
        node.sensors={k:time.monotonic() for k in ('scan','depth','gps')}
        node.state.start(time.monotonic())
        node.goal_handle=Handle()
        node.tick()
        assert not node.done and cancelled==[]
        node.tf_missing_since=time.monotonic()-3
        node.tick()
        assert node.done and not node.report['success']
        assert 'TF' in node.report['reason']
        assert cancelled==[True]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_watchdog_blocks_stale_controller_pose_despite_fresh_own_tf():
    import time
    from geometry_msgs.msg import TransformStamped, Twist
    rclpy.init()
    node=Safety()
    output=[];faults=[]
    node.pub=type('Publisher',(),{'publish':lambda self,msg:output.append(msg.linear.x)})()
    node.fault_pub=type('Publisher',(),{'publish':lambda self,msg:faults.append(msg.data)})()
    try:
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='base_link'
        tf.header.stamp=node.get_clock().now().to_msg();tf.transform.rotation.w=1.0
        node.buffer.set_transform(tf,'test')
        node.times={k:time.monotonic() for k in ('scan','depth','gps','cmd')}
        node.cmd=Twist();node.cmd.linear.x=.3;node.route_running=True
        node.controller_received=time.monotonic()
        node.controller_pose=(tf.header.stamp,1.0,0.0)
        node.tick()
        assert output[-1]==0.0 and not faults
        node.controller_bad_since=time.monotonic()-1
        node.tick()
        assert faults and node.failed
        from std_msgs.msg import String
        node.status(String(data='{"state":"running"}'))
        node.controller_pose=(tf.header.stamp,0.0,0.0)
        node.tick()
        assert output[-1]==0.0
    finally:
        node.destroy_node();rclpy.try_shutdown()
