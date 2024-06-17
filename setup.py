from setuptools import find_packages, setup
import glob

package_name = 'map_localize'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),  # Specify the correct location
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nataraj',
    maintainer_email='natarajsudharsan01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "localize=map_localize.localize_node:main",
        ],
    },
)
