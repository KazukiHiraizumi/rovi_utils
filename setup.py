#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rovi_utils'],
    package_dir={'': 'src'},
    scripts=['script/tf_lookup.py','script/tf_euler.py']
)

setup(**d)
