# ============================================================================
#  vSMC/example/vsmc/CMakeLists.txt
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013-2016, Yan Zhou
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

PROJECT(vSMCExample-vsmc CXX)

FUNCTION(VSMC_ADD_HEADER_TEST basepath cond)
    IF(${cond})
        STRING(REPLACE "/" "_" basename "${basepath}")
        CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/src/vsmc.cpp
            ${PROJECT_BINARY_DIR}/src/${basename}.cpp)
        VSMC_ADD_EXECUTABLE(${basename}_hpp
            ${PROJECT_BINARY_DIR}/src/${basename}.cpp)
        ADD_DEPENDENCIES(vsmc ${basename}_hpp)

        ADD_CUSTOM_TARGET(${basename}_hpp-check
            DEPENDS ${basename}_hpp vsmc-files
            COMMAND ${basename}_hpp
            COMMENT "Running ${basename}_hpp"
            WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
        ADD_DEPENDENCIES(vsmc-check ${basename}_hpp-check)
    ENDIF(${cond})
ENDFUNCTION(VSMC_ADD_HEADER_TEST)

VSMC_ADD_EXAMPLE(vsmc)

VSMC_ADD_HEADER_TEST(vsmc/vsmc TRUE)

VSMC_ADD_HEADER_TEST(vsmc/algorithm/algorithm TRUE)
VSMC_ADD_HEADER_TEST(vsmc/algorithm/random_walk TRUE)

VSMC_ADD_HEADER_TEST(vsmc/core/core TRUE)
VSMC_ADD_HEADER_TEST(vsmc/core/monitor      TRUE)
VSMC_ADD_HEADER_TEST(vsmc/core/particle     TRUE)
VSMC_ADD_HEADER_TEST(vsmc/core/sampler      TRUE)
VSMC_ADD_HEADER_TEST(vsmc/core/state_matrix TRUE)
VSMC_ADD_HEADER_TEST(vsmc/core/weight       TRUE)

VSMC_ADD_HEADER_TEST(vsmc/internal/assert  TRUE)
VSMC_ADD_HEADER_TEST(vsmc/internal/basic   TRUE)
VSMC_ADD_HEADER_TEST(vsmc/internal/common  TRUE)
VSMC_ADD_HEADER_TEST(vsmc/internal/defines TRUE)
VSMC_ADD_HEADER_TEST(vsmc/internal/forward TRUE)
VSMC_ADD_HEADER_TEST(vsmc/internal/traits  TRUE)

VSMC_ADD_HEADER_TEST(vsmc/math/math TRUE)
VSMC_ADD_HEADER_TEST(vsmc/math/constants TRUE)
VSMC_ADD_HEADER_TEST(vsmc/math/vmath     TRUE)

VSMC_ADD_HEADER_TEST(vsmc/resample/resample TRUE)
VSMC_ADD_HEADER_TEST(vsmc/resample/algorithm TRUE)
VSMC_ADD_HEADER_TEST(vsmc/resample/index     TRUE)
VSMC_ADD_HEADER_TEST(vsmc/resample/transform TRUE)

VSMC_ADD_HEADER_TEST(vsmc/rng/rng TRUE "MKL")
VSMC_ADD_HEADER_TEST(vsmc/rng/rng_set         TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/seed            TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/u01             TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/u01_sequence    TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/internal/common TRUE)

VSMC_ADD_HEADER_TEST(vsmc/rng/distribution TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/arcsine_distribution       TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/beta_distribution          TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/cauchy_distribution        TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/chi_squared_distribution   TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/dirichlet_distribution     TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/discrete_distribution      TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/exponential_distribution   TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/extreme_value_distribution TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/fisher_f_distribution      TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/gamma_distribution         TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/laplace_distribution       TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/levy_distribution          TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/logistic_distribution      TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/lognormal_distribution     TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/normal_distribution        TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/normal_mv_distribution     TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/pareto_distribution        TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/rayleigh_distribution      TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/student_t_distribution     TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/u01_distribution           TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/uniform_bits_distribution  TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/uniform_real_distribution  TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/weibull_distribution       TRUE)

VSMC_ADD_HEADER_TEST(vsmc/rng/engine TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/aes_ni   ${AESNI_FOUND})
VSMC_ADD_HEADER_TEST(vsmc/rng/counter  TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/mkl      ${MKL_FOUND})
VSMC_ADD_HEADER_TEST(vsmc/rng/philox   TRUE)
VSMC_ADD_HEADER_TEST(vsmc/rng/rdrand   ${RDRAND_FOUND})
VSMC_ADD_HEADER_TEST(vsmc/rng/threefry TRUE)

VSMC_ADD_HEADER_TEST(vsmc/smp/smp TRUE)
VSMC_ADD_HEADER_TEST(vsmc/smp/backend_base TRUE)
VSMC_ADD_HEADER_TEST(vsmc/smp/backend_omp  ${OPENMP_FOUND})
VSMC_ADD_HEADER_TEST(vsmc/smp/backend_seq  TRUE)
VSMC_ADD_HEADER_TEST(vsmc/smp/backend_tbb  ${TBB_FOUND})

VSMC_ADD_HEADER_TEST(vsmc/utility/utility TRUE)
VSMC_ADD_HEADER_TEST(vsmc/utility/aligned_memory TRUE)
VSMC_ADD_HEADER_TEST(vsmc/utility/covariance     TRUE)
VSMC_ADD_HEADER_TEST(vsmc/utility/hdf5           ${HDF5_FOUND})
VSMC_ADD_HEADER_TEST(vsmc/utility/program_option TRUE)
VSMC_ADD_HEADER_TEST(vsmc/utility/progress       TRUE)
VSMC_ADD_HEADER_TEST(vsmc/utility/stop_watch     TRUE)
