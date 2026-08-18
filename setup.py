import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hybraut_nav_colreg_sim'


def models_data_files():
    """Mirror each models/<vessel>/... tree into
    share/<pkg>/models/<vessel>/... .

    Not a flat glob like the other data_files entries: model.config refers
    to its model.sdf by a relative path, and each model.sdf refers to its
    mesh as a relative meshes/<file> URI, so the install layout has to keep
    the same per-vessel directory structure or those references break.
    Walking models/*/ instead of listing vessels by name also means a new
    vessel folder just needs to exist here at build time -- nothing in
    setup.py has to change to pick it up.
    """
    entries = []
    for model_dir in sorted(glob('models/*/')):
        model_dir = model_dir.rstrip('/')
        entries.append((
            os.path.join('share', package_name, model_dir),
            glob(os.path.join(model_dir, '*.config')) +
            glob(os.path.join(model_dir, '*.sdf')),
        ))
        entries.append((
            os.path.join('share', package_name, model_dir, 'meshes'),
            glob(os.path.join(model_dir, 'meshes', '*')),
        ))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.xml')),
    ] + models_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan McKee',
    maintainer_email='ryanmckee47@icloud.com',
    description='Gazebo COLREG scenario simulation environment (multi-vessel worlds, AIS bridge) for testing the hybraut_nav stack',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vessel_ais_bridge = hybraut_nav_colreg_sim.vessel_ais_bridge:main',
            'obstacle_vessel_controler = hybraut_nav_colreg_sim.obstacle_vessel_controler:main',
            'map_publisher = hybraut_nav_colreg_sim.map_publisher:main',
            'scenario_goal_sender = hybraut_nav_colreg_sim.scenario_goal_sender:main',
        ],
    },
)
