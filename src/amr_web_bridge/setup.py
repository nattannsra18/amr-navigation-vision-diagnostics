from glob import glob

from setuptools import find_packages, setup


package_name = 'amr_web_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(
        exclude=['test'],
    ),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml'),
        ),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='nattanns18',
    maintainer_email='nattann.sra18@gmail.com',
    description=(
        'WebSocket Robot Agent connecting the simulated '
        'ROS 2 AMR to the FastAPI control plane.'
    ),
    license='Apache-2.0',
    tests_require=[
        'pytest',
    ],
    entry_points={
        'console_scripts': [
            (
                'web_bridge_node = '
                'amr_web_bridge.web_bridge_node:main'
            ),
        ],
    },
)
