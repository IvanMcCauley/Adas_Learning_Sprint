from setuptools import find_packages, setup
import os

package_name = 'planning101'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/planning101']),
    ('share/planning101', ['package.xml']),
    (os.path.join('share', 'planning101', 'launch'), ['launch/qos_probe.launch.py']),
    (os.path.join('share', 'planning101', 'config'), ['config/qos_probe.yaml']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='mccauleyivan03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qos_probe = planning101.qos_probe:main',
            'pinger = planning101.pinger:main',
        ],
    },
)
