from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# This file runs with `catkin_make`, do not run it manually!
setup_args = generate_distutils_setup(
    packages=['exproblab_assignment1_russo_gabriele'],
    package_dir={'': 'utilities'}
)

setup(**setup_args)
