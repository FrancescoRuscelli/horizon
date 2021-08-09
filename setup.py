import setuptools

# with open("README.md", "r", encoding="utf-8") as fh:
#     long_description = fh.read()

setuptools.setup(
    name="casadi_horizon",
    version="0.0.3",
    author="Francesco Ruscelli",
    author_email="francesco.ruscelli@iit.it",
    description="Library for Trajectory Optimization based on CasADi",
    # long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/FrancescoRuscelli/horizon_gui",
    # project_urls={
    #     "Bug Tracker": "no",
    # },
    # classifiers=[
    #     "Programming Language :: Python :: 3",
    #     "Operating System :: Ubuntu",
    # ],
    # package_dir={"": "horizon"},
    packages=['horizon', 'horizon.utils'],
    install_requires=['casadi', 'numpy'],
    # packages=setuptools.find_packages(),
    python_requires=">=3.6"
    
)
