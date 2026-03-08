from setuptools import setup

package_name = 'local_map_cropper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hexiaoyi',
    maintainer_email='showyhe@outlook.com',
    description='Crop a local submap from a global OccupancyGrid and save it for Nav2 map_server.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crop_map_node = local_map_cropper.crop_map_node:main',
        ],
    },
)