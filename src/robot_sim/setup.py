from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/static', glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pren12',
    maintainer_email='pren12@todo.todo',
    description='PC simulator with web GUI (not used on the robot)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'simulator = robot_sim.simulator:main',
            'pc_bridge = robot_sim.pc_bridge:main',
        ],
    },
)
