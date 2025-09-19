import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'seiretu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imanoob',
    maintainer_email='ymrs1122@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seiretu_node = seiretu.seiretu_node:main',
            'seiretu_gui = seiretu.seiretu_gui:main',
        ],
    },
)
