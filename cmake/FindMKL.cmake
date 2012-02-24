SET (MKL_FOUND TRUE)

SET (MKL_LIB_PATH ${MKL_LIB_PATH} $ENV{MKLROOT}/lib)
SET (MKL_LIB_PATH ${MKL_LIB_PATH} $ENV{MKLROOT}/lib/intel64)
SET (MKL_LIB_PATH ${MKL_LIB_PATH} $ENV{MKLROOT}/lib/ia32)
SET (MKL_LIB_PATH ${MKL_LIB_PATH} /opt/intel/mkl/lib)
SET (MKL_LIB_PATH ${MKL_LIB_PATH} /opt/intel/mkl/lib/intel64)
SET (MKL_LIB_PATH ${MKL_LIB_PATH} /opt/intel/mkl/lib/ia32)

FIND_PATH (MKL_LIB_DIR libmkl_core.a PATHS ${MKL_LIB_PATH} ENV LIBRARY_PATH)

IF (${MKL_LIB_DIR} STREQUAL MKL_LIB_DIR-NOTFOUND)
    MESSAGE (STATUS "NOT Found MKL")
    SET (MKL_FOUND FALSE)
ELSE (${MKL_LIB_DIR} STREQUAL MKL_LIB_DIR-NOTFOUND)
    FIND_LIBRARY (MKL_INTERFACE_LIB mkl_intel_lp64 ${MKL_LIB_DIR})
    FIND_LIBRARY (MKL_THREADING_LIB mkl_sequential ${MKL_LIB_DIR})
    FIND_LIBRARY (MKL_COMPUTING_LIB mkl_core       ${MKL_LIB_DIR})
    SET (MKL_LINK_LIBRARIES ${MKL_LINK_LIBRARIES} ${MKL_INTERFACE_LIB})
    SET (MKL_LINK_LIBRARIES ${MKL_LINK_LIBRARIES} ${MKL_THREADING_LIB})
    SET (MKL_LINK_LIBRARIES ${MKL_LINK_LIBRARIES} ${MKL_COMPUTING_LIB})
    MESSAGE (STATUS "Found MKL: ${MKL_LINK_LIBRARIES}")
ENDIF (${MKL_LIB_DIR} STREQUAL MKL_LIB_DIR-NOTFOUND)

SET (MKL_INC_PATH ${MKL_INC_PATH} $ENV{MKLROOT}/include)
FIND_PATH (MKL_INCLUDE_DIR mkl_vml.h PATHS ${MKL_INC_PATH} ENV INCLUDE)
IF (${MKL_INCLUDE_DIR} STREQUAL MKL_INCLUDE_DIR-NOTFOUND)
    MESSAGE (STATUS "NOT Found MKL headers")
    SET (MKL_FOUND FALSE)
ELSE (${MKL_INCLUDE_DIR} STREQUAL MKL_INCLUDE_DIR-NOTFOUND)
    INCLUDE_DIRECTORIES (${MKL_INCLUDE_DIR})
    MESSAGE (STATUS "Found MKL headers: ${MKL_INCLUDE_DIR}")
ENDIF (${MKL_INCLUDE_DIR} STREQUAL MKL_INCLUDE_DIR-NOTFOUND)
