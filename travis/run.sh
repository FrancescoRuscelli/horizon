#!/bin/bash
set -ev

# run python or conda distribution
if [ "$DISTRIBUTION" = "python" ]; then
    ./travis/python_run.sh;
fi

if [ "$DISTRIBUTION" = "conda" ]; then
    ./travis/conda_run.sh;
fi
