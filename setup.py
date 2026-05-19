from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- these lines make colcon copy your files into install/ ---
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*.sdf') + glob('models/*.png')),
        (os.path.join('share', package_name, 'models', 'rover'),
            glob('models/rover/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anri',
    maintainer_email='anrihasani9@gmail.com',
    description='Mars rover simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_project.my_node:main',
        ],
    },
)