#!/usr/bin/env python3
"""Мир и карта имеют один источник геометрии."""
import json
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/ab_robot'
SCENE = {
    'start': [-4.0, -3.0], 'finish': [4.0, 3.0],
    'obstacles': [
        {'name': 'central', 'x': 0, 'y': 0, 'sx': 2, 'sy': 2, 'height': 1, 'map': True},
        {'name': 'left', 'x': -2.5, 'y': 1, 'sx': 0.8, 'sy': 2, 'height': 0.8, 'map': True},
        {'name': 'low', 'x': 2, 'y': -1.5, 'sx': 0.7, 'sy': 0.7, 'height': 0.18, 'map': False},
        {'name': 'north', 'x': 1, 'y': 3, 'sx': 1.0, 'sy': 0.8, 'height': 0.8, 'map': True},
    ] + [
        {'name': 'wall_'+str(i), 'x': x, 'y': y, 'sx': sx, 'sy': sy, 'height': 1, 'map': True}
        for i, (x,y,sx,sy) in enumerate([(0,-5,12,.2),(0,5,12,.2),(-6,0,.2,10),(6,0,.2,10)])
    ]
}


def box(name, x, y, sx, sy, height, color, collision=True):
    geom = f'<geometry><box><size>{sx} {sy} {height}</size></box></geometry>'
    return f'''<model name="{name}"><static>true</static><pose>{x} {y} {height/2} 0 0 0</pose><link name="link">
      {f'<collision name="collision">{geom}</collision>' if collision else ''}
      <visual name="visual">{geom}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
    </link></model>'''


def main():
    header = '''<?xml version="1.0"?><sdf version="1.8"><world name="ab_world">
    <physics name="physics" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="ignition-gazebo-navsat-system" name="ignition::gazebo::systems::NavSat"/>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation><latitude_deg>55.75</latitude_deg><longitude_deg>37.61</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>
    <scene><ambient>0.65 0.65 0.65 1</ambient><background>0.82 0.86 0.9 1</background><shadows>false</shadows></scene>
    <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 -0.2 -1</direction><cast_shadows>false</cast_shadows></light>
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View"><ignition-gui><title>Gazebo — А → Б</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></ignition-gui><engine>ogre2</engine><scene>scene</scene><ambient_light>0.65 0.65 0.65</ambient_light><background_color>0.82 0.86 0.9</background_color><camera_pose>-8 -10 12 0 0.8 0.85</camera_pose></plugin>
      <plugin filename="GzSceneManager" name="Scene Manager"><ignition-gui><property type="string" key="state">floating</property><property type="bool" key="showTitleBar">false</property><property type="double" key="width">5</property><property type="double" key="height">5</property></ignition-gui></plugin>
      <plugin filename="InteractiveViewControl" name="View control"><ignition-gui><property type="string" key="state">floating</property><property type="bool" key="showTitleBar">false</property><property type="double" key="width">5</property><property type="double" key="height">5</property></ignition-gui></plugin>
    </gui>'''
    models = [box('ground',0,0,14,12,0.1,'0.8 0.8 0.8 1').replace('0 0 0.05', '0 0 -0.05')]
    for o in SCENE['obstacles']:
        models.append(box(o['name'],o['x'],o['y'],o['sx'],o['sy'],o['height'],'0.65 0.3 0.15 1'))
    for name, point, color in [('zone_A',SCENE['start'],'0.1 0.75 0.25 1'),('zone_B',SCENE['finish'],'0.1 0.35 0.95 1')]:
        models.append(box(name,*point,1,1,.006,color,False))
    (PACKAGE/'worlds/ab_world.sdf').write_text(header+'\n'+'\n'.join(models)+'\n</world></sdf>')
    (PACKAGE/'config/scene.json').write_text(json.dumps(SCENE,indent=2)+'\n')
    width,height,res = 280,240,.05
    ox,oy = -7,-6
    cells = bytearray()
    for row in range(height):
        y=oy+(height-row-.5)*res
        for col in range(width):
            x=ox+(col+.5)*res
            occupied=any(o['map'] and abs(x-o['x'])<=o['sx']/2 and abs(y-o['y'])<=o['sy']/2 for o in SCENE['obstacles'])
            cells.append(0 if occupied else 254)
    (PACKAGE/'maps/ab_map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+cells)
    (PACKAGE/'maps/ab_map.yaml').write_text(yaml.safe_dump(dict(image='ab_map.pgm',mode='trinary',resolution=res,origin=[ox,oy,0],negate=0,occupied_thresh=.65,free_thresh=.196),sort_keys=False))


if __name__ == '__main__':
    main()
