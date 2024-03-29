from setuptools import find_packages, setup
import os
from glob import glob

# Other imports ...


package_name = 'pix_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('pix_vehicle_interface/launch', '*launch.[py]*'))),
        ('lib/' + package_name + '/can_utils', ['pix_vehicle_interface/can_utils/can_receiver.py',
                                                'pix_vehicle_interface/can_utils/can_sender.py']),

        ('lib/' + package_name + '/pix_dataclass', ['pix_vehicle_interface/pix_dataclass/DriveCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/BrakeCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/SteerCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/VehicleCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/WheelTorqueCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/data_utils.py']),

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
            'pix_test = pix_vehicle_interface.test_cmd:main',
        ],
    },
)
