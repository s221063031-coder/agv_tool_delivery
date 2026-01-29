import os
from glob import glob
from setuptools import setup

package_name = 'agv_factory_sim'

setup(
    name=package_name,
    # ... other settings ...
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # CRITICAL LINE: This copies your maps to the install folder
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu-amr',
    maintainer_email='ubuntu-amr@todo.todo',
    description='Tool Cart Delivery AGV for Manufacturing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # We will add control scripts here later
        ],
    },
)
