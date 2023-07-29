import os
from glob import glob
from setuptools import setup

package_name = 'rfp_square_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahb',
    maintainer_email='andreas.bihlmaier@gmx.de',
    description='Package for Robotics for Programmers book providing a Python node that draws a square in turtlesim.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_turtle = rfp_square_turtle.square_turtle:main',
        ],
    },
)
