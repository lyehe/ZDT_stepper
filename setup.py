from setuptools import find_packages, setup

setup(
    name="zdt_stepper",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
)
