# Find RdSeed support
#
# The following variable is set
#
# RD_SEED_FOUND - TRUE if RdSeed is found and work correctly

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindRdSeed.cpp RD_SEED_TEST_SOURCE)

IF (NOT DEFINED RD_SEED_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${RD_SEED_TEST_SOURCE}" RD_SEED_FOUND)
    IF (RD_SEED_FOUND)
        MESSAGE (STATUS "Found RdSeed support")
    ELSE (RD_SEED_FOUND)
        MESSAGE (STATUS "NOT Found RdSeed support")
    ENDIF (RD_SEED_FOUND)
ENDIF (NOT DEFINED RD_SEED_FOUND)
