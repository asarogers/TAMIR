from setuptools import find_packages, setup

package_name = 'rosbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/image_recognition_laptop.py',  'config/tags.yaml',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asaace00',
    maintainer_email='cyberasasoftware@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tracker = rosbot.tracker:main",
            "depth_node = rosbot.processDepth:main",
            "image_node = rosbot.processImage:main",
        ],
    },
)
