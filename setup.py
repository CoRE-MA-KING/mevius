import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mevius'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.*')),
        (
            os.path.join('share', package_name, 'models', 'meshes'),
            glob('models/meshes/*'),
        ),
    ],
    install_requires=['setuptools', 'torch'],
    zip_safe=True,
    maintainer='ma-king',
    maintainer_email='ma-king@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mevius_main = mevius.mevius_main:main',
            'can_communication = mevius.nodes.can_communication:main',
            'sim_communication = mevius.nodes.sim_communication:main',
        ],
    },
)
