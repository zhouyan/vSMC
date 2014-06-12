# Find Intel Cilk Plus support
#
# The following variable is set
#
# CILK_FOUND - TRUE if Cilk Plus is found and work correctly. Currently only
#              Intel compiler has this feature

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindCilk.cpp CILK_TEST_SOURCE)

IF (NOT DEFINED CILK_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${CILK_TEST_SOURCE}" CILK_FOUND)
    IF (CILK_FOUND)
        MESSAGE (STATUS "Found Cilk Plus support")
    ELSE (CILK_FOUND)
        MESSAGE (STATUS "NOT Found Cilk Plus support")
    ENDIF (CILK_FOUND)
ENDIF (NOT DEFINED CILK_FOUND)
