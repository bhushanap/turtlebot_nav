from setuptools import setup

package_name = 'team69_navigate_to_goal'

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
    maintainer='richard',
    maintainer_email='richard@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goToGoal = team69_navigate_to_goal.goToGoal:main',
            'getObjectRange = team69_navigate_to_goal.getObjectRange:main',
            'view_image_raw = team69_navigate_to_goal.view_image_raw:main'
        ],
    },
)
