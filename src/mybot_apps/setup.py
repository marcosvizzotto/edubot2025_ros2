from setuptools import find_packages, setup

package_name = 'mybot_apps'

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
    maintainer='marcos',
    maintainer_email='marcosvizzotto@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'square = mybot_apps.square:main',
            'joint_delta = mybot_apps.joint_delta:main',
            'straight_hold = mybot_apps.straight_hold_heading:main',
        ],
    },
)
