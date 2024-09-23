from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_project_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'model'),
glob(os.path.join('model', '*.*'))),
        (os.path.join('share', package_name, 'model/meshes'), 
glob(os.path.join('model/meshes', '*.*'))),
        (os.path.join('share', package_name, 'model/urdf'),
glob(os.path.join('model/urdf', '*.*'))),
        (os.path.join('share', package_name, 'config'),
glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas',
    maintainer_email='diaz.lu18@gmail.com',
    description='PAQUETE DE PRUEBA',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'say_hi = my_project_pkg.say_hi:main',
            'my_publisher = my_project_pkg.publisher:main',
            'my_subscriber = my_project_pkg.subscriber:main',
            'my_robot_controller = my_project_pkg.my_robot_controller:main'
        ],
    },
)
