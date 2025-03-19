from setuptools import setup

package_name = 'px4_ros_com' 

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'parametric_mission', 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabrielbfranca@gmail.com',
    description='Parametric mission node for PX4',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'parametric_mission = parametric_mission:main', 
            'keyboard_control = keyboard_control:main',
        ],
    },
)