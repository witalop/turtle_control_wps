from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_control_wps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='witalo',
    maintainer_email='witalopietler@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_control = turtle_control_wps.turtle_control:main',
            'actuator = turtle_control_wps.actuator:main',
            'conductor = turtle_control_wps.conductor:main'
        ],
    },
)
