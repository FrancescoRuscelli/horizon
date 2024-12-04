#!/bin/bash -i
set -ev
# DOES NOT WORK (cannot activate a conda environment from here)
# A simple script to initialize the conda environment
ENVIRONMENT_NAME=hori
PACKAGE_NAME=horizon
USER=francesco_ruscelli
# sourcing base conda path to activate environment
source $(conda info --base)/etc/profile.d/conda.sh

echo -e "activating conda environment $ENVIRONMENT_NAME .." 
conda activate $ENVIRONMENT_NAME
echo -e "done."

echo -e "building package $PACKAGE_NAME .." 

mkdir ~/conda-bld
conda config --set anaconda_upload no
export CONDA_BLD_PATH=~/conda-bld
conda build -c conda-forge -c francesco_ruscelli -c robostack .

echo -e "done."

# if push is not a tag, do not upload
if [ -z $TRAVIS_TAG ]; then
    echo "Not a tag build, will not upload to conda";
else
    find $CONDA_BLD_PATH/ -name *.tar.bz2 | while read file
    do
        echo -e "file uploaded: $file"
        anaconda -t $ANACONDA_TOKEN upload -u $USER $file
    done
fi
