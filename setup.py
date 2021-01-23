import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pymycobot",
    version="0.0.1",
    author="Tetsuya SAITO",
    author_email="saito.tetsuya@gmail.com",
    description="Python API for myCobot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/3110/mycobot-python",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=["pyserial", "six"],
    python_requires=">=2.7, !=3.0.*, !=3.1.*, !=3.2.*, !=3.3.*, !=3.4.*",
)
