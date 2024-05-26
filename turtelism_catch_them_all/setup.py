from setuptools import setup
from glob import glob
import os

package_name = 'turtelism_catch_them_all'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='illia',
    maintainer_email='ilyaaladin762@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_controller = turtelism_catch_them_all.turtle_controller:main',
            'turtle_spawner = turtelism_catch_them_all.turtle_spawner:main'
        ],
    },
)
