from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'can_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['can_bridge/CanReceiver.py']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JEON YANG HO',
    maintainer_email='yhjeon@avgenius.kr',
    description='Can Receive/Send and parse to publish or subscribe to Topic',
    license='Apache--2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'CanReceiverNode = can_bridge.CanReceiverNode:main',
        ],
    },
)
