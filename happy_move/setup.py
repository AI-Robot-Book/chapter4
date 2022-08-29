from setuptools import setup

package_name = 'happy_move'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kosei Demura',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='A happy move package',
    license='Apache License 2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'happy_move_node = happy_move.happy_move_node:main',
        ],
    },
)
