from setuptools import find_packages, setup

package_name = 'tamir'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
             'config/tags.yaml',
            'launch/startup.launch.py',
            'launch/playmusic.launch.py',
            'sound/535_Silent.mp3',
            'sound/anti_dog.mp3',
            'sound/experiment.mp3',
            'sound/high_pitch.mp3',
            'config/toast_view.rviz',
        ]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asaace00',
    maintainer_email='asaace00@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'camera_localizer = tamir.camera_localizer:main',
            'tamir_interface = tamir.tamir_interface:main',
            'music_node = tamir.playmusic:main',
            'vision = tamir.vision:main'
        ],
    },
)
