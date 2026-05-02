from setuptools import find_packages, setup

package_name = 'gs_tower_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nolan Kessler',
    maintainer_email='kesslnol@oregonstate.edu',
    description='Provides pan/tilt control for groundstation antenna.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "antenna_control = gs_tower_control.control_script:main"
        ],
    },
)
