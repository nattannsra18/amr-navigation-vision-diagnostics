from setuptools import find_packages, setup


package_name = 'amr_vision'


setup(
    name=package_name,
    version='1.0.0',
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
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='nattanns18',
    maintainer_email='nattann.sra18@gmail.com',
    description=(
        'RGB-D perception, ArUco detection, pose estimation, '
        'and marker TF for the simulated AMR.'
    ),
    license='Apache-2.0',
    tests_require=[
        'pytest',
    ],
    entry_points={
        'console_scripts': [
            (
                'aruco_detector = '
                'amr_vision.aruco_detector:main'
            ),
        ],
    },
)
