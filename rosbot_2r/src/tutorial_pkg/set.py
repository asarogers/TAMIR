from setuptools import setup

package_name = 'tutorial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['my_python_node'],  # Add your Python module names here
    install_requires=['setuptools'],
    zip_safe=True,
    author='husarion',
    author_email='cyberasasoftware@gmail.com',
    maintainer='husarion',
    maintainer_email='cyberasasoftware@gmail.com',
    description='ROS 2 Python package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_python_node = tutorial_pkg.my_python_node:main',  # Adjust this according to your file structure
        ],
    },
)
