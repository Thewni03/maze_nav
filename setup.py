from glob import glob
from setuptools import find_packages, setup

package_name = 'maze_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('lib/' + package_name, ['scripts/twist_stamper.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thewni',
    maintainer_email='thewni@example.com',
    description='Autonomous navigation of TurtleBot3 Burger in a custom maze (ROS 2 Jazzy)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
