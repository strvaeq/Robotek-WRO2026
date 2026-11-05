from glob import glob

from setuptools import setup

package_name = 'renkay_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renkay Team',
    maintainer_email='rod10peru@gmail.com',
    description='Launch files for the Renkay car hardware and challenge nodes.',
    license='MIT',
)
