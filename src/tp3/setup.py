from glob import glob
from setuptools import find_packages, setup

package_name = 'tp3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name, ['groundtruth_camera_left.csv']),
        
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eliseo',
    maintainer_email='eli.feuli37@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [  
            'features = tp3.features:main',
            'matchings = tp3.matchings:main',
            'ransac = tp3.ransac:main',
            'triangulation = tp3.triangulation:main',
            'disparity = tp3.disparidad:main',
            'pose = tp3.pose:main',
            'dense-reconstruction = tp3.dense_resconstruction:main',
            'bag-control = tp3.cmd.bag_control:main',
            'view-cameras = tp3.cmd.view_cameras:main',
            'view-features = tp3.cmd.view_features:main',
            'view-matchings = tp3.cmd.view_matchings:main',
            'view-matchings30 = tp3.cmd.view_matchings30:main',
            'view-matchings-ransac = tp3.cmd.view_matchings_ransac:main',
            'view-epilines = tp3.cmd.view_epilines:main',
            'view-disparity = tp3.cmd.view_disparity:main',
            'plot-cam0-path = tp3.cmd.plot_cam0_path:main',
            'plot-cams-pose = tp3.cmd.plot_cams_pose:main',
            'plot-cam0-pose-PnP = tp3.cmd.plot_cam0_pose_PnP:main',
            'plot-map = tp3.cmd.plot_map:main',
            'plot-map-dense = tp3.cmd.plot_map_dense:main',

        ],
    },
)
