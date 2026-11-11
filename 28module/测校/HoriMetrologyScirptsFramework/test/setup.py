# -*- coding: utf-8 -*-
"""
Filename: setup.py
Author: Jianming.Liu
Date: 2026/4/10 
Description:
 
"""
from setuptools import setup
from Cython.Build import cythonize
import numpy

setup(
    ext_modules=cythonize("test.pyx"),
    include_dirs=[numpy.get_include()]
)
