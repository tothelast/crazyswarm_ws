import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'crazyflie_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # These lines now work because of the imports at the top
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smartlab',
    maintainer_email='smartlab@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'], # Note: I corrected 'extras_require' to 'tests_require' as 'pytest' is a test dependency.
    entry_points={
        'console_scripts': [
            'hello_world = crazyflie_test.hello_world:main',
            'simple_takeoff = crazyflie_test.simple_takeoff:main'
        ],
    },
)
