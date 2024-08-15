from setuptools import find_packages, setup

package_name = 'behaviour_tree'

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
            'rotate_360 = behaviour_tree.rotaterobot:main',
            'laser_data = behaviour_tree.laserdatareading:main',
            'behaviour_tree = behaviour_tree.behaviour_tree:main'
        ],
    },
)
