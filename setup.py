import os
from glob import glob
from setuptools import setup
package_name = 'nanomob_sim'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'models', 'pedestrian'), glob('models/pedestrian/*')),
        (os.path.join('share', package_name, 'models', 'bus'), glob('models/bus/*')),
        (os.path.join('share', package_name, 'models', 'fence'), glob('models/fence/*')),
        (os.path.join('share', package_name, 'models', 'spectator'), glob('models/spectator/*')),
        (os.path.join('share', package_name, 'models', 'car'), glob('models/car/*')),
        (os.path.join('share', package_name, 'models', 'parked_row'), glob('models/parked_row/*')),
        (os.path.join('share', package_name, 'models', 'e5_lanes'), glob('models/e5_lanes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'scenarios'), glob('config/scenarios/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.json')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'safety_controller = nanomob_sim.safety_controller:main',
        'scenario_runner = nanomob_sim.scenario_runner:main',
        'plot_results = nanomob_sim.plot_results:main',
        'slope_plant = nanomob_sim.slope_plant:main',
    ]},
)
