FUNCTION (ADD_SMP_EXECUTABLE base header source smp_name)
    STRING (TOUPPER "${smp_name}" SMP)
    STRING (TOLOWER "${smp_name}" smp)
    SET (VSMC_BASE_DEFINE "
#define VSMC_BACKEND_${SMP} 1
#define BASE_STATE   vsmc::State${SMP}
#define BASE_INIT    vsmc::Initialize${SMP}
#define BASE_MOVE    vsmc::Move${SMP}
#define BASE_MONITOR vsmc::MonitorEval${SMP}
#define BASE_PATH    vsmc::PathEval${SMP}
#define BASE_NUMERIC vsmc::Numeric${SMP}
#include <vsmc/smp/adapter.hpp>
#include <vsmc/smp/state.hpp>
#include <vsmc/smp/backend_${smp}.hpp>
#include <vsmc/utility/integrate/numeric_${smp}.hpp>")

    SET (VSMC_BASE_DEFINE_HEADER "${header}-${smp}.hpp")

    CONFIGURE_FILE (
        ${PROJECT_SOURCE_DIR}/${header}.hpp
        ${PROJECT_SOURCE_DIR}/${header}-${smp}.hpp)
    CONFIGURE_FILE (
        ${PROJECT_SOURCE_DIR}/${source}.cpp
        ${PROJECT_SOURCE_DIR}/${source}-${smp}.cpp)

    SET (GENERATED_SOURCE ${GENERATED_SOURCE}
        ${PROJECT_SOURCE_DIR}/${header}-${smp}.hpp
        CACHE INTERNAL "Generated source")
    SET (GENERATED_SOURCE ${GENERATED_SOURCE}
        ${PROJECT_SOURCE_DIR}/${source}-${smp}.cpp
        CACHE INTERNAL "Generated source")

    ADD_EXECUTABLE (${source}-${smp} ${source}-${smp}.cpp)

    IF (${smp} STREQUAL "omp")
        SET_TARGET_PROPERTIES (${source}-${smp} PROPERTIES COMPILE_FLAGS
            "${OpenMP_CXX_FLAGS}")
    ENDIF (${smp} STREQUAL "omp")

    IF (${smp} STREQUAL "std")
        TARGET_LINK_LIBRARIES (${source}-${smp} ${VSMC_THREAD_LINK_LIBRARIES})
    ELSEIF (${smp} STREQUAL "gcd")
        TARGET_LINK_LIBRARIES (${source}-${smp} ${GCD_LINK_LIBRARIES})
    ELSEIF (${smp} STREQUAL "tbb")
        TARGET_LINK_LIBRARIES (${source}-${smp} ${TBB_LINK_LIBRARIES})
    ELSEIF (${smp} STREQUAL "omp")
        TARGET_LINK_LIBRARIES (${source}-${smp} ${OpenMP_LINK_LIBRARIES})
    ENDIF (${smp} STREQUAL "std")

    ADD_DEPENDENCIES (${base} ${source}-${smp})
ENDFUNCTION (ADD_SMP_EXECUTABLE)

FUNCTION (ADD_EXAMPLE base algs)
    FOREACH (smp ${EXAMPLE_EXECUTABLES})
        IF (${smp} STREQUAL "tbb")
            ADD_TBB_RUNTIME(${base})
        ENDIF (${smp} STREQUAL "tbb")
        FOREACH (alg ${algs})
            ADD_SMP_EXECUTABLE(${base} ${base} ${base}-${alg} ${smp})
            TARGET_LINK_LIBRARIES (${base}-${alg}-${smp}
                ${EXAMPLE_LINK_LIBRARIES})
        ENDFOREACH(alg)
    ENDFOREACH (smp)
ENDFUNCTION (ADD_EXAMPLE)


FUNCTION (COPY_FILE basename filename)
    ADD_CUSTOM_COMMAND (
        OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
        DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy
        ${PROJECT_SOURCE_DIR}/${filename} ${PROJECT_BINARY_DIR}/${filename})
    ADD_CUSTOM_TARGET (${basename}-${filename}
        DEPENDS ${PROJECT_BINARY_DIR}/${filename})
    ADD_DEPENDENCIES (${basename}-files ${basename}-${filename})
ENDFUNCTION (COPY_FILE)

FUNCTION (COPY_DIRECTORY basename dirname fullpath)
    FILE (GLOB_RECURSE dirfiles
        ${fullpath}/*.h ${fullpath}/*.hpp
        ${fullpath}/*.c ${fullpath}/*.cpp ${fullpath}/*.cl)
    ADD_CUSTOM_COMMAND (
        OUTPUT ${PROJECT_BINARY_DIR}/${dirname} DEPENDS ${dirfiles}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_directory
        ${fullpath} ${PROJECT_BINARY_DIR}/${dirname})
    ADD_CUSTOM_TARGET (${basename}-${dirname}
        DEPENDS ${PROJECT_BINARY_DIR}/${dirname})
    ADD_DEPENDENCIES (${basename}-files ${basename}-${dirname})
ENDFUNCTION (COPY_DIRECTORY)

FUNCTION (COPY_FILE_OPTIONAL basename filename)
    IF (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
        COPY_FILE (${basename} ${filename})
    ELSE (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
            MESSAGE (STATUS "File: ${basename}: ${filename} not available")
    ENDIF (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
ENDFUNCTION (COPY_FILE_OPTIONAL)
