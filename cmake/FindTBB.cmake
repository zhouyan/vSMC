# Find Intel Threading Building Blocks library
#
# This module can be used to find TBB headers and libraries
#
# The following variables are set
#
# TBB_FOUND          - TRUE if TBB is found and work correctly
# TBB_INCLUDE_DIR    - The directory containing TBB headers, e.g., tbb/tbb.h
# TBB_LINK_LIBRARIES - TBB libraries that shall be linked to
# TBB_DLL            - TBB DLL files (MSVC only)
#
# The following variables affect the behavior of this module
# TBB_ROOT     - The root to the TBB files, e.g., /opt/intel/tbb
#                Under which one can find include, lib or lib/intel64 etc.
# TBB_INC_PATH - The path CMake shall try to find headers first
# TBB_LIB_PATH - The path CMake shall try to find libraries first
# TBB_DLL_PATH - The path CMake shall try to find DLL files first
#                 This variable is only used in MSVC build.
#                 If found, one can use the function ADD_TBB_RUNTIME
#                 to copy the the DLL files into build directory

FUNCTION (ADD_TBB_RUNTIME exe_name)
    IF (MSVC)
        ADD_CUSTOM_COMMAND (
            OUTPUT ${PROJECT_BINARY_DIR}/tbb.dll
            DEPENDS ${TBB_DLL}
            COMMAND ${CMAKE_COMMAND} ARGS -E copy
            ${TBB_DLL_RELEASE} ${PROJECT_BINARY_DIR}/tbb.dll)
        ADD_CUSTOM_TARGET (${exe_name}_tbb_dll
            DEPENDS ${PROJECT_BINARY_DIR}/tbb.dll)
        ADD_DEPENDENCIES (${exe_name} ${exe_name}_tbb_dll)
        ADD_CUSTOM_COMMAND (
            OUTPUT ${PROJECT_BINARY_DIR}/tbb_debug.dll
            DEPENDS ${TBB_DLL_DEBUG}
            COMMAND ${CMAKE_COMMAND} ARGS -E copy
            ${TBB_DLL_DEBUG} ${PROJECT_BINARY_DIR}/tbb_debug.dll)
        ADD_CUSTOM_TARGET (${exe_name}_tbb_debug_dll
            DEPENDS ${PROJECT_BINARY_DIR}/tbb_debug.dll)
        ADD_DEPENDENCIES (${exe_name} ${exe_name}_tbb_debug_dll)
    ENDIF (MSVC)

    IF (XCODE_VERSION)
        ADD_CUSTOM_COMMAND (
            OUTPUT ${PROJECT_BINARY_DIR}/libtbb.dylib
            DEPENDS ${TBB_LINK_LIBRARIES_RELEASE}
            COMMAND ${CMAKE_COMMAND} ARGS -E copy
            ${TBB_LINK_LIBRARIES_RELEASE} ${PROJECT_BINARY_DIR}/libtbb.dylib)
        ADD_CUSTOM_TARGET (${exe_name}_tbb_dylib
            DEPENDS ${PROJECT_BINARY_DIR}/libtbb.dylib)
        ADD_DEPENDENCIES (${exe_name} ${exe_name}_tbb_dylib)
    ENDIF (XCODE_VERSION)
ENDFUNCTION (ADD_TBB_RUNTIME)

IF (NOT TBB_LIB_PATH)
    IF (MSVC)
        IF (CMAKE_CL_64)
            SET (TBB_MSVC_LIB "intel64")
        ELSE (CMAKE_CL_64)
            SET (TBB_MSVC_LIB "ia32")
        ENDIF (CMAKE_CL_64)
        IF (MSVC_VERSION GREATER 1699)
            SET (TBB_MSVC_LIB "${TBB_MSVC_LIB}/vc11")
        ELSEIF (MSVC_VERSION GREATER 1599)
            SET (TBB_MSVC_LIB "${TBB_MSVC_LIB}/vc10")
        ELSEIF (MSVC_VERSION GREATER 1499)
            SET (TBB_MSVC_LIB "${TBB_MSVC_LIB}/vc9")
        ENDIF (MSVC_VERSION GREATER 1699)

        IF (TBB_ROOT)
            SET (TBB_LIB_PATH "${TBB_ROOT}/lib/${TBB_MSVC_LIB}")
        ELSEIF ($ENV{TBBROOT})
            SET (TBB_LIB_PATH "$ENV{TBBROOT}/lib/${TBB_MSVC_LIB}")
        ENDIF (TBB_ROOT)
    ELSE (MSVC)
        IF (TBB_ROOT)
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} ${TBB_ROOT}/lib)
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} ${TBB_ROOT}/lib/intel64)
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} ${TBB_ROOT}/lib/ia32)
        ELSEIF ($ENV{TBBROOT})
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} $ENV{TBBROOT}/lib)
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} $ENV{TBBROOT}/lib/intel64)
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} $ENV{TBBROOT}/lib/ia32)
        ELSE (TBB_ROOT)
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} /opt/intel/tbb/lib)
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} /opt/intel/tbb/lib/intel64)
            SET (TBB_LIB_PATH ${TBB_LIB_PATH} /opt/intel/tbb/lib/ia32)
        ENDIF (TBB_ROOT)
    ENDIF (MSVC)
ENDIF (NOT TBB_LIB_PATH)

IF (NOT TBB_DLL_PATH AND MSVC)
    IF (TBB_ROOT)
        SET (TBB_DLL_PATH "${TBB_ROOT}/bin/${TBB_MSVC_LIB}")
    ELSEIF ($ENV{TBBROOT})
        SET (TBB_DLL_PATH "$ENV{TBBROOT}/bin/${TBB_MSVC_LIB}")
    ENDIF (TBB_ROOT)
ENDIF (NOT TBB_DLL_PATH AND MSVC)

IF (NOT TBB_INC_PATH)
    IF (TBB_ROOT)
        SET (TBB_INC_PATH ${TBB_INC_PATH} ${TBB_ROOT}/include)
    ELSEIF ($ENV{TBBROOT})
        SET (TBB_INC_PATH ${TBB_INC_PATH} $ENV{TBBROOT}/include)
    ENDIF (TBB_ROOT)
ENDIF (NOT TBB_INC_PATH)

IF (NOT DEFINED TBB_LINK_LIBRARIES)
    INCLUDE (FindThreads)
    FIND_LIBRARY (TBB_LINK_LIBRARIES_RELEASE_FOUND tbb
        PATHS ${TBB_LIB_PATH})
    FIND_LIBRARY (TBB_LINK_LIBRARIES_DEBUG_FOUND
        tbb_debug PATHS ${TBB_LIB_PATH})
    IF (TBB_LINK_LIBRARIES_RELEASE_FOUND AND TBB_LINK_LIBRARIES_DEBUG_FOUND)
        SET (TBB_LINK_LIBRARIES
            optimized ${TBB_LINK_LIBRARIES_RELEASE_FOUND}
            debug ${TBB_LINK_LIBRARIES_DEBUG_FOUND} ${CMAKE_THREAD_LIBS_INIT}
            CACHE STRING "Link to TBB")
        SET (TBB_LINK_LIBRARIES_RELEASE ${TBB_LINK_LIBRARIES_RELEASE_FOUND}
            ${CMAKE_THREAD_LIBS_INIT} CACHE STRING "Link to TBB Release")
        SET (TBB_LINK_LIBRARIES_DEBUG ${TBB_LINK_LIBRARIES_DEBUG_FOUND}
            ${CMAKE_THREAD_LIBS_INIT} CACHE STRING "Link to TBB Debug")
        MESSAGE (STATUS "Found TBB libraries: ${TBB_LINK_LIBRARIES}")
    ELSEIF (TBB_LINK_LIBRARIES_RELEASE_FOUND)
        SET (TBB_LINK_LIBRARIES ${TBB_LINK_LIBRARIES_RELEASE_FOUND}
            ${CMAKE_THREAD_LIBS_INIT} CACHE STRING "Link to TBB")
        SET (TBB_LINK_LIBRARIES_RELEASE ${TBB_LINK_LIBRARIES_RELEASE_FOUND}
            ${CMAKE_THREAD_LIBS_INIT} CACHE STRING "Link to TBB Release")
        MESSAGE (STATUS "Found TBB libraries: ${TBB_LINK_LIBRARIES}")
    ELSE (TBB_LINK_LIBRARIES_RELEASE_FOUND AND TBB_LINK_LIBRARIES_DEBUG_FOUND)
        MESSAGE (STATUS "NOT Found TBB libraries")
    ENDIF (TBB_LINK_LIBRARIES_RELEASE_FOUND AND TBB_LINK_LIBRARIES_DEBUG_FOUND)
ENDIF (NOT DEFINED TBB_LINK_LIBRARIES)

IF (NOT DEFINED TBB_INCLUDE_DIR)
    FIND_PATH (TBB_INCLUDE_DIR tbb/tbb.h PATHS ${TBB_INC_PATH} ENV CPATH)
    IF (TBB_INCLUDE_DIR)
        MESSAGE (STATUS "Found TBB headers: ${TBB_INCLUDE_DIR}")
    ELSE (TBB_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found TBB headers")
    ENDIF (TBB_INCLUDE_DIR)
ENDIF (NOT DEFINED TBB_INCLUDE_DIR)

IF (TBB_LINK_LIBRARIES AND TBB_INCLUDE_DIR)
    SET (TBB_BASIC_FOUND TRUE)
ELSE (TBB_LINK_LIBRARIES AND TBB_INCLUDE_DIR)
    SET (TBB_BASIC_FOUND FALSE)
ENDIF (TBB_LINK_LIBRARIES AND TBB_INCLUDE_DIR)

SET (TBB_TEST_SOURCE "
#include <iostream>
#include <cassert>

#if defined(__clang__) && !defined(_LIBCPP_VERSION) && (__GLIBCXX__ < 20100429)
#ifndef TBB_USE_CAPTURED_EXCEPTION
#define TBB_USE_CAPTURED_EXCEPTION 1
#endif
#endif // __clang__

#include <tbb/tbb.h>

class say
{
    public :

    template <typename IntType>
    void operator() (const tbb::blocked_range<IntType> &block) const
    {
        IntType sum = 0;
        for (IntType i = block.begin(); i != block.end(); ++i)
            sum += i;
        std::cout << sum << std::endl;
    }
};

class sum
{
    public :

    sum () : res_(0) {}

    sum (const sum &other, tbb::split) : res_(0) {}

    void join (const sum &other)
    {
        res_ += other.res_;
    }

    template <typename IntType>
    void operator() (const tbb::blocked_range<IntType> &block)
    {
        long r = res_;
        for (IntType i = block.begin(); i != block.end(); ++i)
            r += i;
        res_ = r;
    }

    long res () const
    {
        return res_;
    }

    private :
    long res_;
};

int main ()
{
    tbb::parallel_for(tbb::blocked_range<int>(0, 100), say());
    sum s;
    tbb::parallel_reduce(tbb::blocked_range<int>(0, 100), s);
    assert(s.res() == 4950);
    return 0;
}
")

IF (TBB_BASIC_FOUND AND NOT MSVC AND NOT DEFINED TBB_TEST_SOURCE_RUNS)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_INCLUDES  ${CMAKE_REQUIRED_INCLUDES})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES}
        ${TBB_INCLUDE_DIR})
    IF (TBB_LINK_LIBRARIES_DEBUG)
        SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
            ${TBB_LINK_LIBRARIES_DEBUG})
    ELSE (TBB_LINK_LIBRARIES_DEBUG)
        SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
            ${TBB_LINK_LIBRARIES})
    ENDIF (TBB_LINK_LIBRARIES_DEBUG)
    CHECK_CXX_SOURCE_RUNS ("${TBB_TEST_SOURCE}" TBB_TEST_SOURCE_RUNS)
    IF (TBB_TEST_SOURCE_RUNS)
        MESSAGE (STATUS "Found TBB")
        SET (TBB_FOUND TRUE CACHE BOOL "Found TBB")
    ELSE (TBB_TEST_SOURCE_RUNS)
        MESSAGE (STATUS "NOT Found TBB")
        SET (TBB_FOUND FALSE CACHE BOOL "NOT Found TBB")
    ENDIF (TBB_TEST_SOURCE_RUNS)
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
ENDIF (TBB_BASIC_FOUND AND NOT MSVC AND NOT DEFINED TBB_TEST_SOURCE_RUNS)

IF (MSVC)
    SET (TBB_DLL_PATH ${TBB_DLL_PATH})
    FIND_FILE (TBB_DLL_RELEASE tbb.dll PATHS ${TBB_DLL_PATH})
    FIND_FILE (TBB_DLL_DEBUG tbb_debug.dll PATHS ${TBB_DLL_PATH})
    IF (TBB_DLL_RELEASE AND TBB_DLL_DEBUG)
        SET (TBB_DLL ${TBB_DLL_RELEASE} ${TBB_DLL_DEBUG})
    ENDIF (TBB_DLL_RELEASE AND TBB_DLL_DEBUG)
ENDIF (MSVC)
