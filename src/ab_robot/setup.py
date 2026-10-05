from glob import glob
from setuptools import setup

setup(
    name='ab_robot', version='0.1.0', packages=['ab_robot'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/ab_robot']),
                ('share/ab_robot', ['package.xml'])] +
               [('share/ab_robot/' + folder, glob(folder + '/*'))
                for folder in ('launch', 'config', 'urdf', 'worlds', 'maps')],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='maintainer', maintainer_email='maintainer@example.invalid',
    description='GPS-маршрут в Gazebo Fortress', license='Apache-2.0',
    entry_points={'console_scripts': [
        'waypoints = ab_robot.waypoints:main',
        'audit = ab_robot.audit:main',
        'safety = ab_robot.safety:main',
    ]},
)
