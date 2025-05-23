import os
from glob import glob
from setuptools import find_packages, setup

from setuptools import find_packages, setup

package_name = 'turtle_example_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Learning TF2 in Python',
    maintainer='DevelNor',
    maintainer_email='develnor@gmail.com',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fixed_turtle_tf2_broadcaster = turtle_example_py.fixed_turtle_tf2_broadcaster:main',
            'dynamic_turtle_tf2_broadcaster = turtle_example_py.dynamic_turtle_tf2_broadcaster:main',
        ],
    },
)