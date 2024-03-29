from setuptools import setup
from glob import glob

package_name = 'ur_isaac'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdfs', glob('urdfs/*.urdf')),
        ('share/' + package_name + '/config', glob('config/*.*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubb',
    maintainer_email='2330834a@student.gla.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joint_controller = ur_isaac.joint_controller:main"

        ],
    },
)
