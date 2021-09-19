from setuptools import setup
from glob import glob

package_name = 'beckhoff_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rafael Goulart',
    maintainer_email='rgoulartns@gmail.com',
    description='Provides a tool to communicate with Beckhoff PLC',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'beckhoff_ros_node = beckhoff_ros.beckhoff_ros_node:main'
        ],
    },
)
