cmake_minimum_required(VERSION 3.10)
project(OMPLTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 3.3.4 REQUIRED)
find_package(pybind11 REQUIRED)

# NumPy include path
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
include_directories(${Python3_INCLUDE_DIRS} ${NUMPY_INCLUDE})

# Add compile definitions to suppress NumPy warnings
add_compile_definitions(NPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION)

# Choose which main file to compile:

set(MAIN_SRC src/main.cpp)

# Get all other source files except main.cpp and mainCVAE.cpp
file(GLOB OTHER_SOURCES "src/*.cpp" "Tools/*.cpp")
list(FILTER OTHER_SOURCES EXCLUDE REGEX "src/main.cpp|src/mainCVAE.cpp")

# Combine the main source with others
set(SOURCES ${MAIN_SRC} ${OTHER_SOURCES})

add_executable(OMPLTest ${SOURCES})

# Suppress warnings
target_compile_options(OMPLTest PRIVATE
    -Wno-attributes
    -Wno-deprecated-declarations
    -Wno-cpp
)

# Include directories
target_include_directories(OMPLTest PRIVATE
    include
    Tools
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${NUMPY_INCLUDE}
    ${pybind11_INCLUDE_DIRS} 
)

# Link libraries
target_link_libraries(OMPLTest PRIVATE
    ompl::ompl
    ${Python3_LIBRARIES}
)