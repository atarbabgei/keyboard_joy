from setuptools import find_packages, setup

package_name = 'keyboard_joy'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
        install_requires=[
        'setuptools',
        'pynput', 
    ],
    zip_safe=True,
    maintainer='Atar Babgei',
    maintainer_email='atarbabgei@gmail.com',
    description='A ROS 2 package to simulate joystick inputs with a keyboard.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_node = keyboard_joy.joy_node:main',  # Entry point for your node
        ],
    },
)
