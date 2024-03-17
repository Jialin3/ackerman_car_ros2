from setuptools import find_packages, setup

package_name = 'get_car_pose'

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
    maintainer='tang',
    maintainer_email='tang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'get_car_pose = get_car_pose.get_car_pose:main',
        'test_point_trans = get_car_pose.test_point_trans:main',
        ],
    },
)
