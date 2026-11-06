import os
from ament_index_python.packages import get_package_share_directory

recognition_config = os.path.join(
    get_package_share_directory('dart_track'), 'config', 'recognition_params.yaml')