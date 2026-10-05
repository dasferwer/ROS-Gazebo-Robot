"""Локальные GPS-координаты и конечный автомат маршрута без зависимостей ROS."""
import math
from dataclasses import dataclass


def gps_valid(latitude, longitude):
    if not (math.isfinite(latitude) and math.isfinite(longitude)
            and -85 <= latitude <= 85 and -180 <= longitude <= 180):
        raise ValueError('Некорректные GPS-координаты; область работы до 85° широты')


@dataclass
class GeoReference:
    latitude: float
    longitude: float
    map_x: float = 0.0
    map_y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        gps_valid(self.latitude, self.longitude)
        if not all(math.isfinite(v) for v in (self.map_x, self.map_y, self.yaw)):
            raise ValueError('Привязка map должна быть конечной')
        phi = math.radians(self.latitude)
        e2 = 6.69437999014e-3
        d = math.sqrt(1 - e2 * math.sin(phi)**2)
        self.north_scale = 6378137.0 * (1 - e2) / d**3
        self.east_scale = 6378137.0 / d * math.cos(phi)

    def to_map(self, latitude, longitude):
        gps_valid(latitude, longitude)
        east = math.radians(longitude - self.longitude) * self.east_scale
        north = math.radians(latitude - self.latitude) * self.north_scale
        if math.hypot(east, north) > 1000:
            raise ValueError('Локальная аппроксимация ограничена радиусом 1 км')
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.map_x + c*east - s*north, self.map_y + s*east + c*north

    def to_gps(self, x, y):
        dx, dy = x - self.map_x, y - self.map_y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        east, north = c*dx + s*dy, -s*dx + c*dy
        if not math.isfinite(east + north) or math.hypot(east, north) > 1000:
            raise ValueError('Точка вне локальной области 1 км')
        return (self.latitude + math.degrees(north/self.north_scale),
                self.longitude + math.degrees(east/self.east_scale))


def validate_route(points):
    if not points:
        raise ValueError('Маршрут пуст')
    for point in points:
        gps_valid(float(point['latitude']), float(point['longitude']))
        if not math.isfinite(float(point.get('yaw', 0))):
            raise ValueError('Курс должен быть конечным')
    return points


class RouteState:
    def __init__(self, count, timeout):
        if count < 1 or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Нужны точки и положительный тайм-аут')
        self.count, self.timeout = count, timeout
        self.index = 0
        self.started = None
        self.finished = self.success = False

    def start(self, now):
        if self.finished or self.started is not None:
            raise RuntimeError('Маршрут уже завершён или цель активна')
        self.started = now

    def expired(self, now):
        return self.started is not None and now - self.started >= self.timeout

    def result(self, succeeded):
        if self.finished or self.started is None:
            raise RuntimeError('Нет активной цели')
        self.started = None
        if not succeeded:
            self.finished = True
            return
        self.index += 1
        self.finished = self.index == self.count
        self.success = self.finished


def command_allowed(now, times, terminal, sensor_timeout, command_timeout):
    return not terminal and now-times.get('cmd',-math.inf)<command_timeout and all(
        now-times.get(k,-math.inf)<sensor_timeout for k in ('scan','depth','gps'))


def obstacle_contact(first, second):
    return not any(name.split('::',1)[0]=='ground' for name in (first,second))
