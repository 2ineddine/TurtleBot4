import os
from glob import glob
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
        
        # REMOVED: YAML files that don't exist
        # ('share/' + package_name, [
        #     'base2lidar_matrix.yaml',
        #     'Lidar2camera_matrix.yaml', 
        #     'base_to_lidar.yaml'
        # ]),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='2ineddine',
    maintainer_email='ririr23@gmail.com',
    description='direct application of the EKF',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera = camera_odom_fusion.camera_treatement:main',
            'ekf = camera_odom_fusion.ekf_treatment:main'
        ],
    },
)
