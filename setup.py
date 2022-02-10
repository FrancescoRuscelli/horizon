import setuptools
from setuptools.command.install import install
from setuptools.command.develop import develop
import os, sys

def _pre_install(dirname):

        # create a build dir and run 'make generate_python_package'
        current_dir = os.getcwd()
        os.makedirs(dirname, exist_ok=True)
        build_dir = current_dir + '/' + dirname
        import subprocess
        try:
            p = subprocess.check_output(["cmake", "-DCMAKE_BUILD_TYPE=Release", "../horizon/cpp"], cwd=build_dir)
        except subprocess.CalledProcessError as e:
            raise 

        try:
            p = subprocess.check_output(["make", "generate_python_package", "-j8"], cwd=build_dir)
        except subprocess.CalledProcessError as e:
            raise

class CustomInstall(install):
    # called when installing normally (pip install .)
    def run(self):
        dir_name = 'temp_build'
        _pre_install(dir_name)
        install.run(self)
        
class CustomDevInstallCommand(develop):
    # called when installing in editable mode (pip install . -e)
    def run(self):
        dir_name = 'temp_build'
        _pre_install(dir_name)
        develop.run(self)

setuptools.setup(
    name="casadi_horizon",
    version="0.3.0",
    author="Francesco Ruscelli",
    author_email="francesco.ruscelli@iit.it",
    description="Library for Trajectory Optimization based on CasADi",
    long_description_content_type="text/markdown",
    url="https://github.com/FrancescoRuscelli/horizon",
    packages=['horizon', 'horizon.utils', 'horizon.solvers', 'horizon.transcriptions', 'horizon.examples', 'horizon.ros'],
    install_requires=['casadi', 'numpy', 'matplotlib', 'scipy', 'casadi-kin-dyn', 'rospkg'],
    python_requires=">=3.6",
    cmdclass={'install': CustomInstall,
              'develop': CustomDevInstallCommand},
    ext_modules=[
        setuptools.Extension(
            name="ilqrext", sources=[]
        ),
        setuptools.Extension(
            name="sqpext", sources=[]
        )
    ]
)
