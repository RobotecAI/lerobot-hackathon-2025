from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lerobot_o3de'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kdabrowski',
    maintainer_email='kacper.dabrowski@robotec.ai',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_states = lerobot_o3de.joint_states:main',
            'put_ball_in_basket_service = lerobot_o3de.put_ball_in_basket_service:main'
        ],
    },
)
