from setuptools import setup
import os
from glob import glob

package_name = 'asterassembly_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Important : inclure le dossier urdf, launch, meshes, config
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilyana',
    maintainer_email='ilyana@todo.todo',
    description='Robot description package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'quasistatic_walker = asterassembly_description.quasistatic_walker:main',
        'servo_bridge = asterassembly_description.ros2_serial_bridge:main',
        'aster_ihm = asterassembly_description.aster_control_center:main',
    ],
},


)

