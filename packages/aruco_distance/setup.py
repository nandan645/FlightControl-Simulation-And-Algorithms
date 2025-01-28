from setuptools import setup

package_name = 'aruco_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for ArUco marker distance calculation',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_distance_node = aruco_distance.aruco_distance_node:main',
        ],
    },
)
