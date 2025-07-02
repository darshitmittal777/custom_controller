from setuptools import find_packages, setup

package_name = 'custom_controller'

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
    maintainer='darshit',
    maintainer_email='darshit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['move_to = custom_controller.move_to:main',
                            'take_input = custom_controller.take_input:main',
                            'take_input_2 = custom_controller.take_input_2:main',
                            'rotate_robot = custom_controller.rotate_robot:main',
                            'move_trajectory = custom_controller.move_trajectory:main',
                            'max_min_coordinates = custom_controller.max_min_coordinates:main',
                            'follow8 = custom_controller.follow8:main',
                            'simple9 = custom_controller.simple9:main',
                            'spiral = custom_controller.spiral:main',
                            'spiral_real = custom_controller.spiral_real:main',
                            'pure_pursuit = custom_controller.pure_pursuit:main',
                            'pure_pursuit_real = custom_controller.pure_pursuit_real:main',
                            'pure_pursuit_multi = custom_controller.pure_pursuit_multi:main'
        ],
    },
)
