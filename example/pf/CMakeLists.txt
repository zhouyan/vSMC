# ============================================================================
#  vSMC/example/pf/CMakeLists.txt
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

PROJECT(vSMCExample-pf CXX)
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

SET(PF_STATE smp)
IF (VSMC_MPI_FOUND)
    SET(PF_STATE ${PF_STATE}; mpi)
ENDIF (VSMC_MPI_FOUND)

FUNCTION(ADD_PF_SMP_CHECK backend)
    ADD_CUSTOM_TARGET(pf_smp_${backend}-check
        DEPENDS pf_smp_${backend} pf-files
        COMMAND pf_smp_${backend}
        "pf.data" "pf_smp_${backend}" ">>pf.out"
        COMMENT "Running pf_smp_${backend}"
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
    ADD_DEPENDENCIES(pf_smp-check pf_smp_${backend}-check)
ENDFUNCTION(ADD_PF_SMP_CHECK smp)

FUNCTION(ADD_PF_MPI_CHECK backend)
    ADD_CUSTOM_TARGET(pf_mpi_${backend}-check
        DEPENDS pf_mpi_${backend} pf-files
        COMMAND ${VSMC_MPIEXEC}
        ${VSMC_MPIEXEC_NUMPROC_FLAG} "2"
        ${VSMC_MPIEXEC_PREFLAGS}
        ${PROJECT_BINARY_DIR}/pf_mpi_${backend}
        ${VSMC_MPIEXEC_POSTFLAGS}
        "pf.data" "pf_mpi_${backend}" ">>pf.out"
        COMMENT "Running pf_mpi_${backend}"
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
    ADD_DEPENDENCIES(pf_mpi-check pf_mpi_${backend}-check)
ENDFUNCTION(ADD_PF_MPI_CHECK base smp)

ADD_SMP_EXAMPLE(pf "${PF_STATE}" "HDF5")

COPY_FILE(pf pf.data)
COPY_FILE(pf pf.R)

ADD_CUSTOM_TARGET(pf-check)
ADD_CUSTOM_TARGET(pf_smp-check)
ADD_DEPENDENCIES(pf-check pf_smp-check)
IF (VSMC_MPI_FOUND)
    ADD_CUSTOM_TARGET(pf_mpi-check)
    ADD_DEPENDENCIES(pf-check pf_mpi-check)
ENDIF (VSMC_MPI_FOUND)
ADD_DEPENDENCIES(check pf-check)

FOREACH (backend ${SMP_EXECUTABLES})
    ADD_PF_SMP_CHECK(${backend})
    IF (VSMC_MPI_FOUND)
        ADD_PF_MPI_CHECK(${backend})
    ENDIF (VSMC_MPI_FOUND)
ENDFOREACH (backend ${SMP_EXECUTABLES})

IF (OPENCL_FOUND)
    COPY_FILE(pf pf_cl.cl)
    ADD_VSMC_EXECUTABLE(pf_cl ${PROJECT_SOURCE_DIR}/src/pf_cl.cpp
        "OCL" "HDF5" "MKL")
    ADD_DEPENDENCIES(pf pf_cl)
    ADD_DEPENDENCIES(example_cl pf_cl)
    ADD_CUSTOM_TARGET(pf_cl-check
        DEPENDS pf_cl pf-files
        COMMAND pf_cl "pf.data" "pf_cl" "-I ${VSMC_INCLUDE_DIR}" ">>pf_cl.out"
        COMMENT "Running pf_cl"
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
    ADD_DEPENDENCIES(pf-check pf_cl-check)

    IF (VSMC_MPI_FOUND)
        ADD_VSMC_EXECUTABLE(pf_cl_mpi ${PROJECT_SOURCE_DIR}/src/pf_cl.cpp
            "OCL" "HDF5" "MKL" "MPI")
        SET_TARGET_PROPERTIES(pf_cl_mpi PROPERTIES COMPILE_FLAGS
            "-DVSMC_PF_CL_MPI")
        ADD_DEPENDENCIES(pf pf_cl_mpi)
        ADD_DEPENDENCIES(example_cl pf_cl_mpi)
        ADD_DEPENDENCIES(example_mpi pf_cl_mpi)
        ADD_CUSTOM_TARGET(pf_cl_mpi-check
            DEPENDS pf_cl_mpi pf-files
            COMMAND ${VSMC_MPIEXEC}
            ${VSMC_MPIEXEC_NUMPROC_FLAG} "2"
            ${VSMC_MPIEXEC_PREFLAGS}
            ${PROJECT_BINARY_DIR}/pf_cl_mpi
            ${VSMC_MPIEXEC_POSTFLAGS}
            "pf.data" "pf_cl_mpi" "-I ${VSMC_INCLUDE_DIR}" ">>pf_cl.out"
            COMMENT "Running pf_cl_mpi"
            WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
        ADD_DEPENDENCIES(pf-check pf_cl_mpi-check)
    ENDIF (VSMC_MPI_FOUND)
ENDIF (OPENCL_FOUND)

ADD_CUSTOM_TARGET(pf-pdf
    DEPENDS pf-files pf-check
    COMMAND Rscript "${PROJECT_BINARY_DIR}/pf.R"
    COMMENT "Plotting pf-check results"
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
ADD_DEPENDENCIES(pdf pf-pdf)
