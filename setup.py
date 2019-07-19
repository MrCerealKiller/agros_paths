#!/usr/bin/env python
""" Agros Path Planning Python Setup"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['agros_paths'],
    package_dir={'': 'src'}
)

setup(**d)
