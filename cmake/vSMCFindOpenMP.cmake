# ============================================================================
#  cmake/vSMCFindOpenMP.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/vSMCFindOpenMP.cpp
    VSMC_OPENMP_TEST_SOURCE)

INCLUDE (FindOpenMP)
IF (OPENMP_FOUND AND NOT DEFINED VSMC_OPENMP_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    IF (NOT MSVC)
        SET (OpenMP_LINK_LIBRARIES ${OpenMP_CXX_FLAGS}
            CACHE STRING "OpenMP Link flags")
    ENDIF (NOT MSVC)
    SET (SAFE_CMAKE_REQUIRED_FLAGS     ${CMAKE_REQUIRED_FLAGS})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (CMAKE_REQUIRED_FLAGS ${SAFE_CMAKE_REQUIRED_FLAGS} ${OpenMP_CXX_FLAGS})
    IF (OpenMP_LINK_LIBRARIES)
        SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
            ${OpenMP_LINK_LIBRARIES})
    ENDIF (OpenMP_LINK_LIBRARIES)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_OPENMP_TEST_SOURCE}" VSMC_OPENMP_FOUND)
    SET (CMAKE_REQUIRED_FLAGS     ${SAFE_CMAKE_REQUIRED_FLAGS})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
ENDIF (OPENMP_FOUND AND NOT DEFINED VSMC_OPENMP_FOUND)
