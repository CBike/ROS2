from setuptools import find_packages, setup
from glob import glob

package_name = 'pix_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('lib/' + package_name + '/can_utils', ['pix_vehicle_interface/can_utils/can_receiver.py',
                                                'pix_vehicle_interface/can_utils/can_sender.py']),

        ('lib/' + package_name + '/pix_dataclass', ['pix_vehicle_interface/pix_dataclass/data_utils.py',
                                                    'pix_vehicle_interface/pix_dataclass/ThrottleCommandData.py',
                                                    'pix_vehicle_interface/pix_dataclass/BrakeCommandData.py',
                                                    'pix_vehicle_interface/pix_dataclass/SteeringCommandData.py',
                                                    'pix_vehicle_interface/pix_dataclass/GearCommandData.py',
                                                    'pix_vehicle_interface/pix_dataclass/ParkCommandData.py',
                                                    'pix_vehicle_interface/pix_dataclass/VehicleModeCommandData.py']),
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
            'pix_interface_rpt = pix_vehicle_interface.pix_interface_rpt_node:main',
            'pix_interface_cmd = pix_vehicle_interface.pix_interface_cmd_node:main',
            'pix_test_control = pix_vehicle_interface.test_cmd:main',
        ],
    },
)
