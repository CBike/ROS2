from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pix_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['pix_vehicle_interface/can_utils/can_receiver.py']),
        ('lib/' + package_name, ['pix_vehicle_interface/can_utils/can_sender.py']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JEON YANG HO',
    maintainer_email='yhjeon@avgenius.kr',
    description='pix vehicle interface by CAN',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'pix_interface_rpt = pix_vehicle_interface.pix_interface_rpt_node:main'
        ],

    },
)
