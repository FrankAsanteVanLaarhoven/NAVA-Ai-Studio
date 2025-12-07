from setuptools import setup

package_name = 'univarm_drive_bridge_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'aiohttp', 'sseclient-py'],
    zip_safe=True,
    maintainer='Univarm',
    maintainer_email='ops@univarm.local',
    description='Bridge between Univarm backend and ROS2 topics',
    entry_points={'console_scripts': ['drive_bridge = univarm_drive_bridge_py.drive_bridge:main']},
)
