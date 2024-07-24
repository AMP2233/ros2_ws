from setuptools import find_packages, setup

package_name = 'py_srvcli'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_node = py_srvcli.service_node:main',
            'client_node = py_srvcli.client_node:main',
            'client_subscription_node = py_srvcli.client_subcription_node:main',
            'service_practice = py_srvcli.service_practice:main',
            'service_practice_b = py_srvcli.service_practice_b:main',
        ],
    },
)
