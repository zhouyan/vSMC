# ============================================================================
#  cmake/vSMCFindThread.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distributed under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find C++11 <thread> support
#
# The following variable is set
#
# CXX11LIB_THREAD_FOUND - TRUE if C++11 <thread> is found and work correctly

IF (DEFINED CXX11LIB_THREAD_FOUND)
    RETURN ()
ENDIF (DEFINED CXX11LIB_THREAD_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindCXX11LibThread.cpp
    CXX11LIB_THREAD_TEST_SOURCE)

INCLUDE (FindThreads)
INCLUDE (CheckCXXSourceRuns)
SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRAREIS})
SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT})
CHECK_CXX_SOURCE_RUNS ("${CXX11LIB_THREAD_TEST_SOURCE}" CXX11LIB_THREAD_FOUND)
SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRAREIS})
