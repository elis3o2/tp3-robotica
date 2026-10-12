from glob import glob
from setuptools import find_packages, setup

package_name = 'tp3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch',
        glob('launch/*.launch.py')),
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
            'stereo-pose = tp3.stereo_pose:main',
            'dense-reconstruction = tp3.desnse_reconstruction:main',
            'plot-features = tp3.plots.plot_features:main',
            'plot-matchings = tp3.plots.plot_matchings:main',
            'plot-matchings30 = tp3.plots.plot_matchings30:main',
            'plot-matchings-ransac = tp3.plots.plot_matchings_ransac:main',
            'plot-epilines = tp3.plots.plot_epilines:main',
            'plot-disparity = tp3.plots.plot_disparity:main',
            'plot-pose = tp3.plots.plot_pose:main',
            'plot-visual-odometry = tp3.plots.plot_visual_odometry:main',
            'plot-stereo-pose = tp3.plots.plot_stereo_pose:main',
        ],
    },
)
