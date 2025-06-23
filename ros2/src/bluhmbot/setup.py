import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bluhmbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name,  'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name,  'src/description'), glob('src/description/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stefan Bluhm',
    maintainer_email='bluhmbot@clacee.eu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_management_node = battery_management.battery_management:main',
            'twist_to_motors_node = differential_drive.twist_to_motors:main',
            'pid_velocity_node = differential_drive.pid_velocity:main',
            'diff_tf_node = differential_drive.diff_tf:main',
            'teleop = bluhmbot.teleop:main'
        ],
    },
)
