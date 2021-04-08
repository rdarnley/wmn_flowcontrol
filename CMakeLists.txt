cmake_minimum_required(VERSION 2.8.3)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE Debug)

# Set CPP Standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GTSAM
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

# Include Directories
include_directories(include src config)

# Declaration of package required
find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

# Add Executables
add_executable(wmn_sim src/wmn_sim.cpp src/factors.cpp src/solver.cpp)
target_link_libraries(wmn_sim gtsam ${YAMLCPP_LIBRARIES}) # python2.7 ${PYTHON_LIBRARIES})

add_executable(sim src/sim.cpp src/solve.cpp src/factor.cpp)
target_link_libraries(sim gtsam ${YAMLCPP_LIBRARIES}) # python2.7 ${PYTHON_LIBRARIES})

find_package(kindr)
include_directories(${kindr_INCLUDE_DIRS})

find_package(ct_optcon)
include_directories(${ct_optcon_INCLUDE_DIRS})