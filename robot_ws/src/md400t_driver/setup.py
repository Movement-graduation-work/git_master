from setuptools import setup

package_name = 'md400t_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='MD400T cmd_vel motor driver',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi4_cmdvel_motor_node = md400t_driver.rpi4_cmdvel_motor_node:main',
        ],
    },
)
