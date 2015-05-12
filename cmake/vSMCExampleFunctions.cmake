# ============================================================================
#  vSMC/vSMCExample/cmake/vSMCExampleFunctions.cmake
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013-2015, Yan Zhou
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# ============================================================================

FUNCTION (ADD_VSMC_EXECUTABLE exe src)
    ADD_EXECUTABLE(${exe} ${src})

    IF (DEFINED VSMC_LINK_LIBRARIES)
        TARGET_LINK_LIBRARIES (${exe} ${VSMC_LINK_LIBRARIES})
    ENDIF (DEFINED VSMC_LINK_LIBRARIES)

    GET_TARGET_PROPERTY (compile_flags ${exe} COMPILE_FLAGS)
    IF (NOT compile_flags)
        UNSET (compile_flags)
    ENDIF (NOT compile_flags)

    GET_TARGET_PROPERTY (link_flags ${exe} LINK_FLAGS)
    IF (NOT link_flags)
        UNSET (link_flags)
    ENDIF (NOT link_flags)

    FOREACH (arg ${ARGN})
        IF (${arg} STREQUAL "MPI" AND VSMC_MPI_FOUND)
            TARGET_LINK_LIBRARIES (${exe} ${VSMC_MPI_LINK_LIBRARIES})
            SET (compile_flags "${compile_flags} ${MPI_CXX_COMPILE_FLAGS}")
            SET (link_flags "${link_flags} ${MPI_CXX_LINK_FLAGS}")
        ENDIF (${arg} STREQUAL "MPI" AND VSMC_MPI_FOUND)

        IF (${arg} STREQUAL "OCL" AND OPENCL_FOUND)
            TARGET_LINK_LIBRARIES (${exe} ${OpenCL_LINK_LIBRARIES})
        ENDIF (${arg} STREQUAL "OCL" AND OPENCL_FOUND)

        IF (${arg} STREQUAL "HDF5" AND HDF5_FOUND)
            TARGET_LINK_LIBRARIES (${exe} ${HDF5_LIBRARIES})
        ENDIF (${arg} STREQUAL "HDF5" AND HDF5_FOUND)

        IF (${arg} STREQUAL "U01" AND TESTU01_FOUND)
            TARGET_LINK_LIBRARIES (${exe} ${TestU01_LINK_LIBRARIES})
        ENDIF (${arg} STREQUAL "U01" AND TESTU01_FOUND)

        IF (${arg} STREQUAL "TBB" AND TBB_FOUND)
            TARGET_LINK_LIBRARIES (${exe} ${TBB_LINK_LIBRARIES})
        ENDIF (${arg} STREQUAL "TBB" AND TBB_FOUND)

        IF (${arg} STREQUAL "OMP" AND OPENMP_FOUND)
            SET (compile_flags "${compile_flags} ${OpenMP_CXX_FLAGS}")
            TARGET_LINK_LIBRARIES (${exe} ${OpenMP_LINK_LIBRARIES})
        ENDIF (${arg} STREQUAL "OMP" AND OPENMP_FOUND)
    ENDFOREACH (arg ${ARGN})

    IF (compile_flags)
        SET_TARGET_PROPERTIES (${exe} PROPERTIES COMPILE_FLAGS
            "${compile_flags}")
    ENDIF (compile_flags)

    IF (link_flags)
        SET_TARGET_PROPERTIES (${exe} PROPERTIES LINK_FLAGS
            "${link_flags}")
    ENDIF (link_flags)
ENDFUNCTION (ADD_VSMC_EXECUTABLE)

FUNCTION (ADD_SMP_EXECUTABLE base header source smp_name)
    STRING (TOUPPER "${smp_name}" SMP)
    STRING (TOLOWER "${smp_name}" smp)

    IF (EXISTS ${PROJECT_SOURCE_DIR}/include/${header}.hpp)
        CONFIGURE_FILE (
            ${PROJECT_SOURCE_DIR}/include/${header}.hpp
            ${PROJECT_BINARY_DIR}/include/${header}_${smp}.hpp)
    ENDIF (EXISTS ${PROJECT_SOURCE_DIR}/include/${header}.hpp)
    CONFIGURE_FILE (
        ${PROJECT_SOURCE_DIR}/src/${source}.cpp
        ${PROJECT_BINARY_DIR}/src/${source}_${smp}.cpp)

    IF (${source} MATCHES "_mpi")
        ADD_VSMC_EXECUTABLE (${source}_${smp}
            ${PROJECT_BINARY_DIR}/src/${source}_${smp}.cpp
            "${SMP}" "MKL" "RT" "MPI" ${ARGN})
        ADD_DEPENDENCIES (example_mpi ${source}_${smp})
    ELSE (${source} MATCHES "_mpi" ${ARGN})
        ADD_VSMC_EXECUTABLE (${source}_${smp}
            ${PROJECT_BINARY_DIR}/src/${source}_${smp}.cpp
            "${SMP}" "MKL" "RT" ${ARGN})
    ENDIF (${source} MATCHES "_mpi")
    ADD_DEPENDENCIES (${base} ${source}_${smp})
    ADD_DEPENDENCIES (example_${smp} ${source}_${smp})
ENDFUNCTION (ADD_SMP_EXECUTABLE)

FUNCTION (ADD_SMP_EXAMPLE base algs)
    ADD_CUSTOM_TARGET (${base})
    ADD_CUSTOM_TARGET (${base}-files)
    ADD_DEPENDENCIES (${base} ${base}-files)
    ADD_DEPENDENCIES (example ${base})
    FOREACH (alg ${algs})
        ADD_CUSTOM_TARGET (${base}_${alg})
        FOREACH (smp ${SMP_EXECUTABLES})
            ADD_SMP_EXECUTABLE (${base} ${base} ${base}_${alg} ${smp} ${ARGN})
            ADD_DEPENDENCIES (${base}_${alg} ${base}_${alg}_${smp})
        ENDFOREACH (smp)
    ENDFOREACH (alg)
ENDFUNCTION (ADD_SMP_EXAMPLE)

FUNCTION (COPY_FILE basename filename)
    IF (UNIX)
        ADD_CUSTOM_COMMAND (
            OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
            DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
            COMMAND ${CMAKE_COMMAND} ARGS -E create_symlink
            ${PROJECT_SOURCE_DIR}/${filename}
            ${PROJECT_BINARY_DIR}/${filename})
    ELSE (UNIX)
        ADD_CUSTOM_COMMAND (
            OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
            DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
            COMMAND ${CMAKE_COMMAND} ARGS -E copy
            ${PROJECT_SOURCE_DIR}/${filename}
            ${PROJECT_BINARY_DIR}/${filename})
    ENDIF (UNIX)
    ADD_CUSTOM_TARGET (${basename}-${filename}
        DEPENDS ${PROJECT_BINARY_DIR}/${filename})
    ADD_DEPENDENCIES (${basename}-files ${basename}-${filename})
ENDFUNCTION (COPY_FILE)

FUNCTION (COPY_FILE_OPTIONAL basename filename)
    IF (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
        COPY_FILE (${basename} ${filename})
    ELSE (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
            MESSAGE (STATUS "File: ${basename}: ${filename} not available")
    ENDIF (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
ENDFUNCTION (COPY_FILE_OPTIONAL)

FUNCTION (ADD_HEADER_EXECUTABLE basepath cond)
    IF (${cond})
        STRING (REPLACE "/" "_" basename "${basepath}")
        IF (EXISTS ${PROJECT_SOURCE_DIR}/src/${basename}.cpp)
            ADD_VSMC_EXECUTABLE (${basename}_hpp
                ${PROJECT_SOURCE_DIR}/src/${basename}.cpp ${ARGN})
        ELSE (EXISTS ${PROJECT_SOURCE_DIR}/src/${basename}.cpp)
            CONFIGURE_FILE (${PROJECT_SOURCE_DIR}/src/vsmc.cpp
                ${PROJECT_BINARY_DIR}/src/${basename}.cpp)
            ADD_VSMC_EXECUTABLE (${basename}_hpp
                ${PROJECT_BINARY_DIR}/src/${basename}.cpp ${ARGN})
        ENDIF (EXISTS ${PROJECT_SOURCE_DIR}/src/${basename}.cpp)
        ADD_DEPENDENCIES (vsmc ${basename}_hpp)
    ENDIF (${cond})
ENDFUNCTION (ADD_HEADER_EXECUTABLE)
