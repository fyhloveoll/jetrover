import os
from glob import glob
from setuptools import setup

package_name = 'jr_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fyh',
    maintainer_email='fanyuheng14@gmail.com',
    description='YOLO detection + depth->3D grasp target (eye-in-hand) for JetRover; '
                'publishes annotated raw+compressed stream for remote viewing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector = jr_vision.detector:main',
            'grasp = jr_vision.grasp:main',
            'scene = jr_vision.scene:main',
        ],
    },
)
