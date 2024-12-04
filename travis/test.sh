#!/bin/bash
set -ev

if [ "$DISTRIBUTION" = "python" ]; then
    cd $HOME
    nosetests3 -w $HORIZON_DIR/horizon/tests
fi