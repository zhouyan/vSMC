# ============================================================================
#  vSMC/example/rng/CMakeLists.txt
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013,2014, Yan Zhou
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

PROJECT (vSMCExample-rng CXX)
INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)

ADD_CUSTOM_TARGET (rng)
ADD_DEPENDENCIES (example rng)

ADD_CUSTOM_TARGET (rng-files)
ADD_DEPENDENCIES (rng rng-files)

ADD_CUSTOM_TARGET (rng-check)
ADD_DEPENDENCIES (check rng-check)

ADD_CUSTOM_TARGET (rng-pdf)
ADD_DEPENDENCIES (pdf rng-pdf)

FUNCTION (ADD_RNG_TEST name)
    ADD_EXECUTABLE (rng_${name} ${PROJECT_SOURCE_DIR}/src/rng_${name}.cpp)
    TARGET_LINK_LIBRARIES (rng_${name} ${VSMC_LINK_LIBRARIES})
    ADD_DEPENDENCIES (rng rng_${name})
    ADD_CUSTOM_TARGET (rng_${name}-check
        COMMAND rng_${name} ">>rng.out"
        DEPENDS rng_${name}
        COMMENT "Running rng_${name}"
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
    ADD_DEPENDENCIES (rng-check rng_${name}-check)
ENDFUNCTION (ADD_RNG_TEST)

FUNCTION (ADD_U01_TEST name)
    IF (TESTU01_FOUND)
        ADD_EXECUTABLE (rng_${name}_testu01
            ${PROJECT_SOURCE_DIR}/src/rng_${name}_testu01.cpp)
        TARGET_LINK_LIBRARIES (rng_${name}_testu01 ${VSMC_LINK_LIBRARIES})
        TARGET_LINK_LIBRARIES (rng_${name}_testu01 ${TestU01_LINK_LIBRARIES})
        ADD_DEPENDENCIES (rng rng_${name}_testu01)
    ENDIF (TESTU01_FOUND)
ENDFUNCTION (ADD_U01_TEST)

ADD_RNG_TEST(dist)

ADD_RNG_TEST(std)
ADD_U01_TEST(std)

ADD_RNG_TEST(philox)
ADD_U01_TEST(philox)

ADD_RNG_TEST(threefry)
ADD_U01_TEST(threefry)

ADD_RNG_TEST(xorshift)
ADD_U01_TEST(xorshift)

IF (AESNI_FOUND)
    ADD_RNG_TEST(aes)
    ADD_U01_TEST(aes)
    ADD_RNG_TEST(ars)
    ADD_U01_TEST(ars)
    ADD_EXECUTABLE (rng_aes_validation
        ${PROJECT_SOURCE_DIR}/src/rng_aes_validation.cpp)
    TARGET_LINK_LIBRARIES (rng_aes_validation ${VSMC_LINK_LIBRARIES})
    ADD_DEPENDENCIES (rng rng_aes_validation)
ENDIF (AESNI_FOUND)

IF (RDRAND_FOUND)
    ADD_RNG_TEST(rdrand)
    ADD_U01_TEST(rdrand)
ENDIF (RDRAND_FOUND)

IF (VSMC_RANDOM123_FOUND)
    ADD_RNG_TEST(r123)
    ADD_U01_TEST(r123)
ENDIF (VSMC_RANDOM123_FOUND)

IF (GSL_FOUND)
    ADD_RNG_TEST(gsl)
    TARGET_LINK_LIBRARIES (rng_gsl ${GSL_LINK_LIBRARIES})
    TARGET_LINK_LIBRARIES (rng_gsl ${GSL_CBLAS_LINK_LIBRARIES})
    IF (TESTU01_FOUND)
        ADD_U01_TEST(gsl)
        TARGET_LINK_LIBRARIES (rng_gsl_testu01 ${GSL_LINK_LIBRARIES})
        TARGET_LINK_LIBRARIES (rng_gsl_testu01 ${GSL_CBLAS_LINK_LIBRARIES})
    ENDIF (TESTU01_FOUND)
ENDIF (GSL_FOUND)

IF (MKL_FOUND)
    ADD_RNG_TEST(mkl)
    TARGET_LINK_LIBRARIES (rng_mkl ${MKL_LINK_LIBRARIES})
    IF (TESTU01_FOUND)
        ADD_U01_TEST(mkl)
        TARGET_LINK_LIBRARIES (rng_mkl_testu01 ${MKL_LINK_LIBRARIES})
    ENDIF (TESTU01_FOUND)
ENDIF (MKL_FOUND)

IF (OPENCL_FOUND AND RANDOM123_FOUND)
    COPY_FILE (rng rng_cl.cl)
    COPY_FILE (rng rng_cl.R)
    ADD_EXECUTABLE (rng_cl ${PROJECT_SOURCE_DIR}/src/rng_cl.cpp)
    TARGET_LINK_LIBRARIES (rng_cl ${VSMC_LINK_LIBRARIES})
    TARGET_LINK_LIBRARIES (rng_cl ${OpenCL_LINK_LIBRARIES})
    ADD_DEPENDENCIES (rng rng_cl)
    ADD_DEPENDENCIES (example_cl rng_cl)
    ADD_CUSTOM_TARGET (rng_cl-check
        DEPENDS rng_cl rng-files
        COMMAND rng_cl
        "vSMCOpenCLDefault" "vSMCOpenCLDefault" "vSMCOpenCLDefault"
        "float" "10000"
        "-I ${VSMC_INCLUDE_DIR} -I ${Random123_INCLUDE_DIR}" ">>rng_cl.out"
        COMMENT "Running rng_cl"
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
    ADD_CUSTOM_TARGET (rng_cl-pdf
        DEPENDS rng_cl-check
        COMMAND Rscript "${PROJECT_BINARY_DIR}/rng_cl.R" ">>rng_cl.out"
        COMMENT "Plotting rng_cl-check results"
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
    ADD_DEPENDENCIES (rng-pdf rng_cl-pdf)
ENDIF (OPENCL_FOUND AND RANDOM123_FOUND)