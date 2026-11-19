from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'demo_supervisor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='Demo supervisor: STT/Qwen → Stage1/2/4 → PPO → PickPlaceCommand',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'demo_supervisor_node = demo_supervisor.demo_supervisor_node:main',
        ],
    },
)
