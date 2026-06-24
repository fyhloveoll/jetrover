from setuptools import setup

package_name = 'jr_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fyh',
    maintainer_email='fanyuheng14@gmail.com',
    description='Own teleop for JetRover (keyboard + gamepad, publishes /controller/cmd_vel)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = jr_teleop.keyboard_teleop:main',
            'joy_teleop = jr_teleop.joy_teleop:main',
            'cmd_vel_relay = jr_teleop.cmd_vel_relay:main',
        ],
    },
)
