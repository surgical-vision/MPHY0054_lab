from setuptools import setup

package_name = 'lab02_task03'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zorion96',
    maintainer_email='zorion96@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'lab02_task03_talker = lab02_task03.lab02_task03_pub:main',
        'lab02_task03_listener = lab02_task03.lab02_task03_sub:main',
        ],
    },
)
