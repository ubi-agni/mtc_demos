#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # list of packages to setup
    packages = ['mtc_demos'],
    # specify location of root ('') package dir
    package_dir = {'' : 'python/src'}
)
setup(**d)
