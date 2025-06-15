from setuptools import find_packages, setup

package_name = 'odom_computation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amitk',
    maintainer_email='amitk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'odom_node = odom_computation.odom_node:main',
             'wheel_tick_publisher = odom_computation.wheel_tick_pub:main',
             'imu_publisher = odom_computation.imu_publisher:main',
        ],
    },
)
