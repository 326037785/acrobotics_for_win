from setuptools import setup, find_packages

with open("requirements.txt") as f:
    requirements = f.read().splitlines()

dev_requirements = ["pytest", "pytest-cov", "codecov"]

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="acrobotics",
    version="0.0.6",
    author="Jeroen De Maeyer",
    author_email="jeroen.demaeyer@kuleuven.be",
    description="Primitive robot kinematics and collision checking.",
    url="https://github.com/JeroenDM/acrobotics",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(where="src"),
    install_requires=requirements,
    extras_require={"dev": dev_requirements},
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    package_dir={"": "src"},
    python_requires=">=3.6",
)
