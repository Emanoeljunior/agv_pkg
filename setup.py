## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_qmc = generate_distutils_setup(
    packages=['py_qmc5883l'],
    package_dir={'': 'src'})

setup_mpu = generate_distutils_setup(
    packages=['py_mpu6050'],
    package_dir={'': 'src'})

setup(**setup_qmc)
setup(**setup_mpu)
