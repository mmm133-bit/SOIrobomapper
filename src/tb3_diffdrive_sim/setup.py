from setuptools import find_packages, setup

package_name = 'tb3_diffdrive_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/diffdrive_launch.py']),
        ('share/' + package_name + '/config', ['config/tb3_controller.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/tb3_with_control.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubu',
    maintainer_email='ubu@todo.todo',
    description='TurtleBot3 diff drive sim using ros2_control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
