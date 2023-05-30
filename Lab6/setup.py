from setuptools import setup

package_name = 'team69_final'

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
            'readSign = team69_final.readSign:main',
            'goToGoal = team69_final.goToGoal:main',
            'getObjectRange = team69_final.getObjectRange:main',
            'getWallRange = team69_final.getWallRange:main',
            'view_image_raw = team69_final.view_image_raw:main',
            'view_image_processed = team69_final.view_image_processed:main'
        ],
    },
)
