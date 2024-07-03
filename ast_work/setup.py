from setuptools import find_packages, setup

package_name = 'ast_work'

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
    maintainer='student',
    maintainer_email='kamrankhowaja1999@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'statemachine_node = ast_work.statemachine:main',
            'statemachine_ros_node = ast_work.statemachine_rostechnique:main',
            'rotate_360 = ast_work.rotaterobot:main',
            'laser_data = ast_work.laserdatareading:main',
            'behaviour_tree = ast_work.behaviour_tree:main'
        ],
    },
)
