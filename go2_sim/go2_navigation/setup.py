from setuptools import find_packages, setup

package_name = 'go2_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/navigation_launch.py']),
        ('share/' + package_name + '/params', ['params/nav2_params.yaml']),
        ('share/' + package_name + '/xml' , ['xml/go2_bt.xml']),
        ('share/' + package_name + '/map' , ['map/map.yaml']),
        ('share/' + package_name + '/map' , ['map/map.pgm']),
        ('share/' + package_name, ['launch/navigation_gpt.launch.py']),
        ('share/' + package_name, ['launch/online_async_launch.py']),
        ('share/' + package_name + '/config' , ['config/mapper_params_online_async.yaml']),





    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nalishe',
    maintainer_email='nalishe@gmail.com',
    description='Tgo2_nav',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_tf_publisher = go2_navigation.tf_publisher:main',
            'static_map_publisher = go2_navigation.static_map_publisher:main'
        ],
    },
)
