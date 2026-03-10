from setuptools import find_packages, setup

setup(
    name='hexapod',
    packages=find_packages(include=['hexapod']),
    version='1.0.0',
    description='Library of Hexapod Functions',
    long_description='Library of hexapod functions.',
    author='Nabeel Chowdhury',
    author_email='nabeel.chowdhury@case.edu',
    license='GPL-2.0',
    install_requires=[
        'numpy>=1.22.3', 
        'pyserial>=3.5',
        'adafruit-circuitpython-mcp3xxx>=1.4.7', 
        'inputs>=0.5'
    ],
    # REMOVED: setup_requires=['pytest-runner>=6.0.0'],
    tests_require=[
        'pytest>=7.1.0', 
        'numpy>=1.22.3', 
        'pyserial>=3.5',
        'adafruit-circuitpython-mcp3xxx>=1.4.7'
    ],
    test_suite='tests',
)