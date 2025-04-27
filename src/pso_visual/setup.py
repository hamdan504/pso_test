from setuptools import setup

package_name = 'pso_visual'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pso_visual.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/pso_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='PSO Algorithm Visualization',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pso_visual_node = pso_visual.pso_visual_node:main',
        ],
    },
)
