from setuptools import find_packages, setup

setup(
    name='pyjcnsfranka',
    packages=find_packages(include=['pyjcnsfranka']),
    version='0.1.0',
    description='Python wrapper for JcnsFranka C++ library.',
    author='Konstantin Kholostov',
    license='LGLP',
    install_requires=[],
    setup_requires=[],
    tests_require=[],
    test_suite='',
)
