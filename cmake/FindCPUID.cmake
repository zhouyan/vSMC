# Find CPUID support
#
# The following variable is set
#
# CPUID_FOUND - TRUE if CPUID instruction is found and work correctly

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindCPUID.cpp
    CPUID_TEST_SOURCE)

IF (NOT DEFINED CPUID_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${CPUID_TEST_SOURCE}" CPUID_FOUND)
    IF (CPUID_FOUND)
        MESSAGE (STATUS "Found CPUID support")
    ELSE (CPUID_FOUND)
        MESSAGE (STATUS "NOT Found CPUID support")
    ENDIF (CPUID_FOUND)
ENDIF (NOT DEFINED CPUID_FOUND)
