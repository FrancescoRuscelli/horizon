import setuptools

# with open("README.md", "r", encoding="utf-8") as fh:
#     long_description = fh.read()

setuptools.setup(
    name="horizon_diocane",
    version="0.0.1",
    author="Francesco Ruscelli",
    author_email="francesco.ruscelli@iit.it",
    description="Beh",
    # long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/FrancescoRuscelli/horizon_gui",
    # project_urls={
    #     "Bug Tracker": "no",
    # },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: Ubuntu",
    ],
    # package_dir={"": "classes"},
    packages=['classes', 'custom_css'],
    # packages=setuptools.find_packages(where="classes"),
    python_requires=">=3.6",
)