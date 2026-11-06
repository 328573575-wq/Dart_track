import os
from ament_index_python.packages import get_package_share_directory

real_light_centers = os.path.join(
    get_package_share_directory('dart_track'), 'config', 'real_light_centers.yaml')