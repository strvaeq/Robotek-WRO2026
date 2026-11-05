from setuptools import setup

package_name = 'renkay_obstacle'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renkay Team',
    maintainer_email='rod10peru@gmail.com',
    description='Obstacle Challenge controller: wall following and colour pillar avoidance.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_node = renkay_obstacle.robot_node:main',
            'motor_driver = renkay_obstacle.motor_driver:main',
        ],
    },
)
