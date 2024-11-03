from setuptools import find_packages, setup

package_name = 'simulasensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Igor OS',
    maintainer_email='igor.oliveirasilvestre95@gmail.com',
    description='Pacote ROS2 para simular leitura de sensor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'sensor = simulasensor.sensor:main',
        ],
    },
)