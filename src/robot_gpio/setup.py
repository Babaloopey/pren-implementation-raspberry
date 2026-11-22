from setuptools import find_packages, setup

package_name = 'robot_gpio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pren12',
    maintainer_email='pren12@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'search_button_driver = robot_gpio.search_button_driver:main',
            'bone_button_driver = robot_gpio.bone_button_driver:main',
            'led_driver = robot_gpio.led_driver:main',
            'speaker_driver = robot_gpio.speaker_driver:main',
        ],
    },
)
