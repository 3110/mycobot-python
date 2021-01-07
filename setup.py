import sys
import setuptools

if sys.version_info[:2] < (3, 5):
    print('This mycobot version requires Python 3.5 or later.')
    sys.exit(1)

with open('README.md', 'r', encoding='utf-8') as fh:
    long_description = fh.read()

setuptools.setup(
    name='pymycobot',
    version='0.0.1',
    author='Tetsuya SAITO',
    author_email='saito.tetsuya@gmail.com',
    description='Python API for myCobot',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/3110/mycobot-python',
    packages=setuptools.find_packages(),
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    install_requires=['pyserial'],
    python_requires='>=3.5',
)
