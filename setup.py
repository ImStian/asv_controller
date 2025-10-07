from setuptools import setup
from glob import glob
import os

package_name = 'asv_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/asv_controller.launch.py']),
        ('share/' + package_name, ['circle_radius_3m.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'autograd'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ASV Towing Controller Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asv_controller_node = asv_controller.asv_controller_node:main',
        ],
    },
    python_requires='>=3.6',
    package_dir={'': '.'},
    include_package_data=True,
)
