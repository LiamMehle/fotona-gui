#!/bin/python3
from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy

ext_modules = [
    Extension(
        'main2',
        ['main2.pyx'],
        extra_compile_args=['-fopenmp'],
        extra_link_args=['-fopenmp'],
    )
]
setup(name='tester',
    ext_modules=cythonize(ext_modules),
    include_dirs=[numpy.get_include()],
    compiler_directives={
        'boundscheck': False,
        'language_level': 3,
        'wraparound': False},
    requires=[
        'pyyaml'],
)


