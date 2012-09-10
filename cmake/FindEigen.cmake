IF (NOT Eigen_INCLUDE_DIR)
    SET (Eigen_INC_PATH ${Eigen_INC_PATH} /usr/local/include/eigen3)
    SET (Eigen_INC_PATH ${Eigen_INC_PATH} /usr/include/eigen3)
    FIND_PATH (Eigen_INCLUDE_DIR signature_of_eigen3_matrix_library
        PATHS ${Eigen_INC_PATH} ENV CPATH)
ENDIF (NOT Eigen_INCLUDE_DIR)

IF (Eigen_INCLUDE_DIR)
    MESSAGE (STATUS "Found Eigen headers: ${Eigen_INCLUDE_DIR}")
    SET (Eigen_FOUND TRUE)
ELSE (Eigen_INCLUDE_DIR)
    MESSAGE (STATUS "NOT Found Eigen headers")
    SET (Eigen_FOUND FALSE)
ENDIF (Eigen_INCLUDE_DIR)
