#!/usr/bin/env python3
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages, setup


setup(
    **generate_distutils_setup(
        packages=find_packages("src"),
        package_dir={"": "src"},
    )
)
