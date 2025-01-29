from setuptools import find_packages
from setuptools import setup

setup(
    name='tamir_interface',
    version='0.0.0',
    packages=find_packages(
        include=('tamir_interface', 'tamir_interface.*')),
)
