"""GPS привязывает маршрут к map; Nav2 выполняет каждую цель с тайм-аутом."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import NavSatFix, LaserScan, PointCloud2
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from ab_robot.core import GeoReference, RouteState, validate_route, obstacle_contact


class Waypoints(Node):
    def __init__(self):
        super().__init__('gps_waypoints')
        self.wall_clock = RosClock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter('route_file', '')
        self.declare_parameter('report_file', '/ws/artifacts/route.json')
        self.declare_parameter('tf_timeout_s',.5)
        self.declare_parameter('tf_failure_timeout_s',2.0)
        self.tf_missing_since=None
        self.last_tf_stamp=None
        self.tf_progress=time.monotonic()
        self.cfg = yaml.safe_load(Path(self.get_parameter('route_file').value).read_text())
        self.points = validate_route(self.cfg['waypoints'])
        self.state = RouteState(len(self.points), self.cfg['goal_timeout_s'])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.lifecycle_future = None
        self.nav_ready = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.status_pub = self.create_publisher(String, '/route_status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.sensors = {}
        self.fixes = []
        self.reference = None
        self.goal_handle = None
        self.result_future = None
        self.send_future = None
        self.started = time.monotonic()
        self.report = {'state': 'waiting', 'goals': [], 'success': False}
        self.done = False
        self.create_subscription(NavSatFix, '/gps/fix', self.gps, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', lambda m: self.received('scan'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/depth/points', lambda m: self.received('depth'), qos_profile_sensor_data)
        self.create_subscription(String,'/safety_fault',lambda m:self.finish(False,m.data),
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for topic in ('/contacts/body','/contacts/left','/contacts/right'):
            self.create_subscription(Contacts,topic,self.contact,qos_profile_sensor_data)
        self.create_timer(.1, self.tick, clock=self.wall_clock)
        self.publish()

    def contact(self,msg):
        if any(obstacle_contact(c.collision1.name,c.collision2.name) for c in msg.contacts):
            self.finish(False,'Контакт с препятствием; привод остановлен')

    def received(self, sensor):
        self.sensors[sensor] = time.monotonic()

    def gps(self, msg):
        if msg.status.status < 0 or not math.isfinite(msg.latitude + msg.longitude):
            return
        self.received('gps')
        if self.reference is None:
            self.fixes.append((msg.latitude, msg.longitude))
            self.fixes = self.fixes[-5:]

    def publish(self):
        if rclpy.ok():
            self.status_pub.publish(String(data=json.dumps(self.report, ensure_ascii=False)))

    def finish(self, success, reason):
        if self.done:
            return
        if rclpy.ok() and self.goal_handle is not None and not success:
            self.goal_handle.cancel_goal_async()
        elif rclpy.ok() and self.send_future is not None and not success:
            if self.send_future.done():
                self.cancel_late_goal(self.send_future)
            else:
                self.send_future.add_done_callback(self.cancel_late_goal)
        self.done = True
        self.report.update(state='succeeded' if success else 'failed', success=success,
                           reason=reason, wall_seconds=round(time.monotonic()-self.started, 3))
        self.publish()
        path = Path(self.get_parameter('report_file').value)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.report, ensure_ascii=False, indent=2)+'\n')
        self.get_logger().info(f'Маршрут завершён: {reason}')

    def cancel_late_goal(self,future):
        if not rclpy.ok():
            return
        try:
            handle=future.result()
            if handle.accepted:
                handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f'Ошибка отмены отложенной цели: {exc}')

    def pose(self):
        tf=self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        stamp=rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds
        now=time.monotonic()
        if stamp!=self.last_tf_stamp:
            self.last_tf_stamp=stamp
            self.tf_progress=now
        timeout=self.get_parameter('tf_timeout_s').value
        age=(self.get_clock().now().nanoseconds-stamp)*1e-9
        if not (-.5<=age<timeout and now-self.tf_progress<timeout):
            raise TransformException('Устарел TF локализации')
        return tf

    def tick(self):
        if self.done:
            return
        now = time.monotonic()
        if self.reference is None:
            if self.lifecycle_future is not None and self.lifecycle_future.done():
                try:
                    self.nav_ready = self.lifecycle_future.result().current_state.id == 3
                except Exception:
                    self.nav_ready = False
                self.lifecycle_future = None
            if not self.nav_ready and self.lifecycle_future is None and self.lifecycle.service_is_ready():
                self.lifecycle_future = self.lifecycle.call_async(GetState.Request())
            if now - self.started > self.cfg['startup_timeout_s']:
                self.finish(False, 'Тайм-аут готовности сенсоров, TF или Nav2')
                return
            if not (len(self.fixes) == 5 and all(k in self.sensors for k in ('scan','depth','gps'))
                    and self.nav_ready and self.client.server_is_ready()):
                return
            try:
                tf = self.pose()
            except TransformException:
                return
            anchor = self.cfg['anchor']
            if math.hypot(tf.transform.translation.x-anchor['map_x'],
                          tf.transform.translation.y-anchor['map_y']) > .5:
                return
            latitude = sum(p[0] for p in self.fixes)/5
            longitude = sum(p[1] for p in self.fixes)/5
            expected = GeoReference(anchor['expected_latitude'], anchor['expected_longitude'])
            try:
                east,north = expected.to_map(latitude,longitude)
            except ValueError as exc:
                self.finish(False, str(exc))
                return
            if math.hypot(east,north) > anchor['tolerance_m']:
                self.finish(False, 'GPS не совпадает с опорной точкой старта')
                return
            # Пять живых измерений задают datum; курс ENU задан в конфигурации мира.
            self.reference = GeoReference(latitude,longitude,anchor['map_x'],anchor['map_y'],
                                          self.cfg['datum']['enu_to_map_yaw'])
            self.report['gps_anchor'] = [latitude,longitude]
        if any(now-self.sensors.get(k,0) > self.cfg['sensor_timeout_s'] for k in ('scan','depth','gps')):
            self.finish(False,'Устарели обязательные данные сенсоров')
            return
        try:
            self.pose()
            self.tf_missing_since=None
        except TransformException:
            if self.tf_missing_since is None:
                self.tf_missing_since=now
            elif now-self.tf_missing_since>=self.get_parameter('tf_failure_timeout_s').value:
                self.finish(False,'Потерян или устарел TF локализации; запрошена отмена')
            return
        if self.state.expired(now):
            self.state.result(False)
            self.finish(False, 'Тайм-аут цели; запрошена отмена')
            return
        if self.state.started is None:
            point = self.points[self.state.index]
            try:
                x,y = self.reference.to_map(point['latitude'],point['longitude'])
            except ValueError as exc:
                self.finish(False,str(exc))
                return
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y = x,y
            yaw = point.get('yaw',0.0)
            goal.pose.pose.orientation.z = math.sin(yaw/2)
            goal.pose.pose.orientation.w = math.cos(yaw/2)
            self.state.start(now)
            self.goal_handle = None
            self.result_future = None
            self.report['goals'].append({'index':self.state.index,'target':[x,y],'state':'sent'})
            self.report['state'] = 'running'
            self.send_future = self.client.send_goal_async(goal)
            self.publish()
            return
        if self.send_future is not None and self.send_future.done():
            try:
                self.goal_handle = self.send_future.result()
                if not self.goal_handle.accepted:
                    self.state.result(False)
                    self.finish(False,'Nav2 отклонил цель')
                    return
                self.result_future = self.goal_handle.get_result_async()
            except Exception as exc:
                self.finish(False,f'Ошибка отправки цели: {exc}')
                return
            self.send_future = None
        if self.result_future is not None and self.result_future.done():
            try:
                result = self.result_future.result()
                succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
                tf = self.pose()
                x,y = self.report['goals'][-1]['target']
                error = math.hypot(tf.transform.translation.x-x,tf.transform.translation.y-y)
                succeeded = succeeded and error <= self.cfg['xy_tolerance_m']
                self.report['goals'][-1].update(state='succeeded' if succeeded else 'failed',
                                               action_status=result.status,map_error_m=error)
                self.result_future = None
                self.state.result(succeeded)
                self.publish()
                if self.state.finished:
                    self.finish(self.state.success,'Все точки достигнуты' if self.state.success else 'Nav2 не достиг цели')
            except Exception as exc:
                self.finish(False,f'Ошибка результата: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Waypoints()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.finish(False,'Остановлено пользователем')
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
