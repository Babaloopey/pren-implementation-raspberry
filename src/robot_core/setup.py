from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pren12',
    maintainer_email='pren12@todo.todo',
    description='Behaviour of the dog: state_coordinator (state machine), command_coordinator',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'topic_debugger = robot_core.topic_debugger:main',
            'state_coordinator = robot_core.state_coordinator:main',
            'command_coordinator = robot_core.command_coordinator:main',
        ],
    },
)
