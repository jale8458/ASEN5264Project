#!/bin/bash
# build_and_run.sh
# Bash script to build and run the OMPL SE(2) project on Apple Silicon

# Stop on first error
set -e

# If first flag is "-m", then cmake command will be run


# Name of build directory
BUILD_DIR=build

# Create build directory

# Configure CMake
if [ "$1" == "-m" ]; then
    # Remove old build (optional)
    if [ -d "$BUILD_DIR" ]; then
        echo "Cleaning old build directory..."
        rm -rf "$BUILD_DIR"
    fi
    echo "Calling CMake."
    ####################### NOTE: The python3.12 line is for Macbook because TensorFlow is only supported to 3.12
    # cmake -B build -S . -DCMAKE_OSX_ARCHITECTURES=arm64
    cmake -B build -S . -DPython3_EXECUTABLE=/opt/homebrew/opt/python@3.12/bin/python3.12 -DCMAKE_OSX_ARCHITECTURES=arm64
else
    echo "Skipping CMake. First argument was not -m."
fi
make -C $BUILD_DIR -j$(sysctl -n hw.ncpu)

# Run the executable
cd $BUILD_DIR
echo "Running OMPLTest..."
./OMPLTest
