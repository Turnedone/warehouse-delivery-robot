from setuptools import find_packages, setup

package_name = 'wdr_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Loukas Pavlou',
    maintainer_email='loukas.pavlou11@gmail.com',
    description='Order submission CLI and run metrics for the warehouse delivery robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'submit_orders = wdr_tools.submit_orders:main',
            'metrics = wdr_tools.metrics:main',
        ],
    },
)
