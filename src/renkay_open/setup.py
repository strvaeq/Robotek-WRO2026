from setuptools import setup

package_name = 'renkay_open'

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
    description='Open Challenge controller: cascaded LiDAR + IMU heading control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_node = renkay_open.robot_node:main',
            'motor_driver = renkay_open.motor_driver:main',
        ],
    },
)
