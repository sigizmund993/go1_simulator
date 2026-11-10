from setuptools import setup

package_name = 'go1_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sigizmund993',
    maintainer_email='claudefan007@gmail.com',
    description='Python Robot class for controlling the Unitree Go1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = go1_control.main:main',
            'lidar_drive = go1_control.lidar_drive:main',
        ],
    },
)
