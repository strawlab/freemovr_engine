from setuptools import setup, find_packages
from os import path
from io import open

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.rst'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='freemovr_engine',
    description='the FreemoVR virtual reality engine',
    long_description_content_type='text/x-rst',
    url='https://github.com/strawlab/freemovr_engine',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
)
