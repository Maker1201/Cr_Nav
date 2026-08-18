from setuptools import setup

package_name = 'terrain_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/terrain_perception_launch.py']),
        ('share/' + package_name + '/config', ['config/terrain_perception.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pio',
    maintainer_email='liyuminglyx@163.com',
    description='Near-field terrain hazard detection from front/rear depth cameras.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_perception_node = terrain_perception.terrain_perception_node:main',
        ],
    },
)
