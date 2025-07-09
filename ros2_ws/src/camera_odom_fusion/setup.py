from setuptools import setup
from setuptools import find_packages

package_name = 'camera_odom_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            'camera_odom_fusion/base2lidar_matrix.yaml',
            'camera_odom_fusion/Lidar2camera_matrix.yaml',
            'camera_odom_fusion/base_to_lidar.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zineddine',
    maintainer_email='zbou6599@gmail.com',
    description='Camera + Odometry fusion',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera = camera_odom_fusion.camera_treatement:main',
            'ekf = camera_odom_fusion.ekf_treatment:main'
        ],
    },
)

