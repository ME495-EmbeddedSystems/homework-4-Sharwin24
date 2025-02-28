"""Setup the nubot_nav package."""
from setuptools import find_packages, setup

package_name = 'nubot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'launch/manual_explore.launch.xml',
            'launch/explore.launch.xml',
            'config/nubot_nav_view.rviz',
            'config/nubot_nav2_params.yaml',
            'config/nubot_nav_slam_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sharwin',
    maintainer_email='sharwin24@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore = nubot_nav.explore:main',
        ],
    },
)
