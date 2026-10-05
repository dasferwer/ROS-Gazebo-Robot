#!/usr/bin/env python3
import subprocess
from pathlib import Path
from xml.etree import ElementTree as E
root=Path(__file__).resolve().parents[1]
urdf=subprocess.check_output(['xacro',str(root/'src/ab_robot/urdf/robot.urdf.xacro')])
path=root/'artifacts/robot.urdf'
path.write_bytes(urdf)
sdf=subprocess.check_output(['ign','sdf','-p',str(path)])
(root/'artifacts/robot.sdf').write_bytes(sdf)
model=E.fromstring(sdf).find('model')
for link in model.findall('link'):
    names={c.attrib['name'] for c in link.findall('collision')}
    for sensor in link.findall("sensor[@type='contact']"):
        for ref in sensor.findall('contact/collision'):
            assert ref.text in names, (link.attrib['name'],ref.text,names)
print('Ссылки контактных сенсоров соответствуют SDF')
