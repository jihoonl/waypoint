from setuptools import setup

package_name = 'waypoint_touring'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jihoon Lee',
    maintainer_email='jihoon@floatic.io',
    description='tour waypoints',
    license='propreitary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tour = waypoint_touring.tour:main'
        ],
    },
)
