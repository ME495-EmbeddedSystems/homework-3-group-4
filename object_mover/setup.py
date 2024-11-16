from setuptools import find_packages, setup

package_name = 'object_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # add ;launch files
        ('share/' + package_name + '/launch', ['launch/object_mover.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Khachatryan',
    maintainer_email='davidkh@u.northwestern.edu',
    description='Package to use moveit for moving an item',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_node = object_mover.UserNode:main',
            'pick_node = object_mover.pick_node:pick_entry'
        ],
    },
)
