from setuptools import find_packages, setup

package_name = 'camera_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xjx',
    maintainer_email='xjx@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'perception = camera_detection.perception:main',
          'operation = camera_detection.operation:main',
          'stream = camera_detection.stream:main',
        ],
    },
)
