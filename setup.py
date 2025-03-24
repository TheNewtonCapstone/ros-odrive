from setuptools import find_packages, setup

package_name = 'n_odrive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/newton.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TheNewtonCapstone',
    maintainer_email='the.newton.capstone@gmail.com',
    author='Shami Ivan',
    author_email='shamiivan@gmail.com',
    description='ROS2 package for interfacing with ODrive motor controllers',
    license='GPLv3.0',
    entry_points={
        'console_scripts': [
            'odrive_node = odrive.odrive_node:main'
        ],
    },
)
