from setuptools import setup

package_name = 'dart_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/scene.yaml', 'config/real_light_centers.yaml', 'config/ost.yaml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Standalone synthetic dart scene + ground truth (simulation only).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_world = dart_sim.sim_world:main',
            'sim_viewer = dart_sim.sim_viewer:main',
            'sim_eval = dart_sim.sim_eval:main',
        ],
    },
)
