SET (Eigen_FOUND TRUE)

SET (Eigen_INC_PATH ${Eigen_INC_PATH} /usr/local/include/eigen3)
SET (Eigen_INC_PATH ${Eigen_INC_PATH} /usr/include/eigen3)

FIND_PATH (Eigen_INCLUDE_DIR signature_of_eigen3_matrix_library
    PATHS ${Eigen_INC_PATH} ENV INCLUDE)
IF (${Eigen_INCLUDE_DIR} STREQUAL Eigen_INCLUDE_DIR-NOTFOUND)
    MESSAGE (STATUS "NOT Found Eigen headers")
    SET (Eigen_FOUND FALSE)
ELSE (${Eigen_INCLUDE_DIR} STREQUAL Eigen_INCLUDE_DIR-NOTFOUND)
    MESSAGE (STATUS "Found Eigen headers: ${Eigen_INCLUDE_DIR}")
ENDIF (${Eigen_INCLUDE_DIR} STREQUAL Eigen_INCLUDE_DIR-NOTFOUND)

IF (NOT Eigen_FOUND)
    SET (Eigen_INCLUDE_DIR)
ENDIF (NOT Eigen_FOUND)
