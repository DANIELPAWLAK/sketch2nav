from setuptools import setup
import os
from glob import glob

package_name = 'demo_gazebo'

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "models", "demo"), glob("models/demo/*.sdf")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pawlak',
    maintainer_email='danielpawlak@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
