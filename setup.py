import setuptools

setuptools.setup(
    name="casadi_horizon",
    version="0.2.1",
    author="Francesco Ruscelli",
    author_email="francesco.ruscelli@iit.it",
    description="Library for Trajectory Optimization based on CasADi",
    long_description_content_type="text/markdown",
    url="https://github.com/FrancescoRuscelli/horizon",
    packages=['horizon', 'horizon.utils', 'horizon.solvers', 'horizon.transcriptions', 'horizon.examples', 'horizon.ros'],
    install_requires=['casadi', 'numpy', 'matplotlib', 'scipy', 'casadi-kin-dyn', 'rospkg'],
    python_requires=">=3.6",
    ext_modules=[
        setuptools.Extension(
            name="ilqrext", sources=[]
        ),
        setuptools.Extension(
            name="sqpext", sources=[]
        )
    ]
)
