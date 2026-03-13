from setuptools import find_packages, setup

package_name = 'robot_vision_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jivan',
    maintainer_email='jivan@todo.com',
    description='Python vision nodes for 380Robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_detector_node = robot_vision_py.line_detector_node:main',
            'object_detector_node = robot_vision_py.object_detector_node:main',
            'dummy_camera_node = robot_vision_py.dummy_camera_node:main',
            'teleop_node = robot_vision_py.teleop_node:main',
            'mjpeg_camera_node = robot_vision_py.mjpeg_camera_node:main',
        ],
    },
)
