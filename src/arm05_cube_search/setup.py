from setuptools import setup

package_name = 'arm05_cube_search'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/search.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARM 05',
    maintainer_email='maintainer@example.com',
    description='Autonomous ArUco cube search for ARM 05.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_detector = arm05_cube_search.aruco_detector:main',
            'cube_mapper = arm05_cube_search.cube_mapper:main',
            'search_manager = arm05_cube_search.search_manager:main',
        ],
    },
)
