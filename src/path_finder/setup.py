from setuptools import setup

package_name = 'path_finder'
submodules= "path_finder/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['3.jpg']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayberk',
    maintainer_email='ayberk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "path_publisher = path_finder.path_publisher:main",
        "path_subscriber = path_finder.path_subscriber:main",
        "image_publisher = path_finder.image_publisher:main",
        "image_subscriber = path_finder.image_subscriber:main"
        ],
    },
)
