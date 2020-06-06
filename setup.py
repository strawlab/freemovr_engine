from setuptools import setup, Extension, find_packages
from os import path
from io import open
import numpy
from Cython.Build import cythonize

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='freemovr_engine',
    description='the FreemoVR virtual reality engine',
    long_description_content_type='text/markdown',
    url='https://github.com/strawlab/freemovr_engine',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    ext_modules = cythonize(Extension("PyDisplaySurfaceArbitraryGeometry",["./src/PyDisplaySurfaceArbitraryGeometry.pyx"],
        libraries=["osg","osgUtil","osgDB","jansson"]
        )),
    include_dirs=[numpy.get_include()],
)
