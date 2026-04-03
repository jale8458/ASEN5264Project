cmake_minimum_required(VERSION 3.10)
project(OMPLTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 3.3.4 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(JlCxx REQUIRED)

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
list(FILTER OTHER_SOURCES EXCLUDE REGEX "src/main.cpp|src/jl_lib.cpp")

##### Julia library
add_library(JlOMPL SHARED src/jl_lib.cpp ${OTHER_SOURCES})

# Define absolute path to environment directory
target_compile_definitions(JlOMPL PRIVATE 
    ENV_DIR="${CMAKE_SOURCE_DIR}/environments/"
)

# Include directories
set(COMMON_INCLUDES
    include
    Tools
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${NUMPY_INCLUDE}
    ${pybind11_INCLUDE_DIRS}
)
target_include_directories(JlOMPL PRIVATE ${COMMON_INCLUDES})

# Link libraries
set(COMMON_LIBRARIES
    ompl::ompl
    ${Python3_LIBRARIES}
)
target_link_libraries(JlOMPL PRIVATE ${COMMON_LIBRARIES}
    JlCxx::cxxwrap_julia
)

#/##### OMPL Executable Generation
#/add_executable(OMPL ${MAIN_SRC} ${OTHER_SOURCES})

#/# Suppress warnings
#/target_compile_options(OMPL PRIVATE
#/    -Wno-attributes
#/    -Wno-deprecated-declarations
#/    -Wno-cpp
#/)

#/# Define absolute path to environment directory
#/target_compile_definitions(OMPL PRIVATE 
#/    ENV_DIR="${CMAKE_SOURCE_DIR}/environments/"
#/)

#/# Include directories and libraries
#/target_include_directories(OMPL PRIVATE ${COMMON_INCLUDES})
#/target_link_libraries(OMPL PRIVATE ${COMMON_LIBRARIES})

##### Install Julia library
install(TARGETS
  JlOMPL
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION lib)