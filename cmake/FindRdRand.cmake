# Find RdRand support
#
# The following variable is set
#
# RD_RAND_FOUND - TRUE if RdRand is found and work correctly

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindRdRand.cpp RD_RAND_TEST_SOURCE)

IF (NOT DEFINED RD_RAND_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${RD_RAND_TEST_SOURCE}" RD_RAND_FOUND)
    IF (RD_RAND_FOUND)
        MESSAGE (STATUS "Found RdRand support")
    ELSE (RD_RAND_FOUND)
        MESSAGE (STATUS "NOT Found RdRand support")
    ENDIF (RD_RAND_FOUND)
ENDIF (NOT DEFINED RD_RAND_FOUND)
