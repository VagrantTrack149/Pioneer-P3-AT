from setuptools import find_packages, setup

package_name = 'p3dxros2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/p3dx.wbt', 'worlds/p3at.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/p3dx.urdf', 'resource/p3at.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neil',
    maintainer_email='neil@example.com',
    description='Pioneer 3-AT and 3-DX with Webots and ROS2 Jazzy',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'p3dxnode = p3dxros2.p3dxnode:main',
            'p3atnode = p3dxros2.p3atnode:main',
            'navegador = p3dxros2.navegador_node:main',
        ],
    },
)
