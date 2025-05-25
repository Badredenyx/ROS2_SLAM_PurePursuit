from setuptools import setup

package_name = 'nav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/nav_controller"]),
        ("share/nav_controller", ["package.xml"]),
        ("share/nav_controller/launch", [
            "launch/multi_robot.launch.py",
            # add any other launch scripts here
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'control = nav_controller.control:main'
        ],
    },
)
