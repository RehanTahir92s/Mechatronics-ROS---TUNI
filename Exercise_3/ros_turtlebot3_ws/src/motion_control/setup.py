from setuptools import setup

package_name = 'motion_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bpresa',
    maintainer_email='rehantahir.sabbih@tuni.fi',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'motion_ctrl = motion_control.main:main',
        'recorder = motion_control.recorder:main',
        ],
    },
)
