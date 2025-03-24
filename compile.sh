#!/bin/bash

# Script to compile the VelCtrl project
rm -r $FLAIR_ROOT/flair-build/eric_src/VelCtrl
mkdir -p $FLAIR_ROOT/flair-build/eric_src/VelCtrl
# Change to the build directory
cd $FLAIR_ROOT/flair-build/eric_src/VelCtrl

# Run the cmake script
$FLAIR_ROOT/flair-src/scripts/cmake_codelite_outofsource.sh $FLAIR_ROOT/eric_src/VelCtrl

# Change to the build directory
cd build

# Run make install
make install

echo "VelCtrl project has been compiled and installed successfully!"