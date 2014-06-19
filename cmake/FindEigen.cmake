# Find Eigen
#
# This module can be used to find Eigen headers
#
# The following variables are set
#
# EIGEN_FOUND          - TRUE if Eigen headers are found
# Eigen_INCLUDE_DIR    - The directory containing OpenCL headers
#
# The following variables affect the behavior of this module
#
# Eigen_INC_PATH - The path CMake shall try to find headers first

IF (NOT DEFINED EIGEN_FOUND OR NOT DEFINED Eigen_INCLUDE_DIR)
    FIND_PATH (Eigen_INCLUDE_DIR Eigen/Eigen
        PATHS ${Eigen_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
    FIND_PATH (Eigen_INCLUDE_DIR Eigen/Eigen)
    IF (Eigen_INCLUDE_DIR)
        MESSAGE (STATUS "Found Eigen headers: ${Eigen_INCLUDE_DIR}")
        SET (EIGEN_FOUND TRUE CACHE BOOL "Found Eigen")
    ELSE (Eigen_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found Eigen headers")
        SET (EIGEN_FOUND FALSE CACHE BOOL "Not Found Eigen")
    ENDIF (Eigen_INCLUDE_DIR)
ENDIF (NOT DEFINED EIGEN_FOUND OR NOT DEFINED Eigen_INCLUDE_DIR)
