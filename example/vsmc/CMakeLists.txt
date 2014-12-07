# ============================================================================
#  vSMC/example/vsmc/CMakeLists.txt
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

PROJECT (vSMCExample-vsmc CXX)

ADD_CUSTOM_TARGET (vsmc)
ADD_DEPENDENCIES (example vsmc)

VSMC_HEADER(vsmc/core/core            TRUE)
VSMC_HEADER(vsmc/core/adapter         TRUE)
VSMC_HEADER(vsmc/core/monitor         TRUE)
VSMC_HEADER(vsmc/core/particle        TRUE)
VSMC_HEADER(vsmc/core/path            TRUE)
VSMC_HEADER(vsmc/core/sampler         TRUE)
VSMC_HEADER(vsmc/core/single_particle TRUE)
VSMC_HEADER(vsmc/core/state_matrix    TRUE)
VSMC_HEADER(vsmc/core/state_tuple     ${CXX11LIB_TUPLE_FOUND})
VSMC_HEADER(vsmc/core/weight_set      TRUE)

VSMC_HEADER(vsmc/cxx11/cmath       ${VSMC_CXX11_MATH_FOUND})
VSMC_HEADER(vsmc/cxx11/functional  TRUE)
VSMC_HEADER(vsmc/cxx11/random      TRUE)
VSMC_HEADER(vsmc/cxx11/type_traits TRUE)

VSMC_HEADER(vsmc/gcd/gcd               ${GCD_FOUND})
VSMC_HEADER(vsmc/gcd/dispatch_function ${GCD_FOUND})
VSMC_HEADER(vsmc/gcd/dispatch_group    ${GCD_FOUND})
VSMC_HEADER(vsmc/gcd/dispatch_object   ${GCD_FOUND})
VSMC_HEADER(vsmc/gcd/dispatch_queue    ${GCD_FOUND})
VSMC_HEADER(vsmc/gcd/dispatch_source   ${GCD_FOUND})

VSMC_HEADER(vsmc/integrate/integrate               TRUE)
VSMC_HEADER(vsmc/integrate/is_integrate            TRUE)
VSMC_HEADER(vsmc/integrate/nintegrate_base         TRUE)
VSMC_HEADER(vsmc/integrate/nintegrate_newton_cotes TRUE)

VSMC_HEADER(vsmc/internal/assert   TRUE)
VSMC_HEADER(vsmc/internal/common   TRUE)
VSMC_HEADER(vsmc/internal/compiler TRUE)
VSMC_HEADER(vsmc/internal/config   TRUE)
VSMC_HEADER(vsmc/internal/defines  TRUE)
VSMC_HEADER(vsmc/internal/forward  TRUE)
VSMC_HEADER(vsmc/internal/traits   TRUE)

VSMC_HEADER(vsmc/math/math      TRUE)
VSMC_HEADER(vsmc/math/constants TRUE)
VSMC_HEADER(vsmc/math/cblas     TRUE)
VSMC_HEADER(vsmc/math/vmath     TRUE)

VSMC_HEADER(vsmc/mpi/mpi         ${VSMC_MPI_FOUND})
VSMC_HEADER(vsmc/mpi/backend_mpi ${VSMC_MPI_FOUND})
VSMC_HEADER(vsmc/mpi/mpi_manager ${VSMC_MPI_FOUND})

VSMC_HEADER(vsmc/opencl/opencl              ${OPENCL_FOUND} "OPENCL")
VSMC_HEADER(vsmc/opencl/adapter             ${OPENCL_FOUND} "OPENCL")
VSMC_HEADER(vsmc/opencl/backend_cl          ${OPENCL_FOUND} "OPENCL")
VSMC_HEADER(vsmc/opencl/cl_buffer           ${OPENCL_FOUND} "OPENCL")
VSMC_HEADER(vsmc/opencl/cl_manager          ${OPENCL_FOUND} "OPENCL")
VSMC_HEADER(vsmc/opencl/cl_manip            ${OPENCL_FOUND} "OPENCL")
VSMC_HEADER(vsmc/opencl/cl_query            ${OPENCL_FOUND} "OPENCL")
VSMC_HEADER(vsmc/opencl/cl_setup            ${OPENCL_FOUND} "OPENCL")
VSMC_HEADER(vsmc/opencl/internal/cl_wrapper ${OPENCL_FOUND} "OPENCL")

VSMC_HEADER(vsmc/resample/resample            TRUE)
VSMC_HEADER(vsmc/resample/common              TRUE)
VSMC_HEADER(vsmc/resample/multinomial         TRUE)
VSMC_HEADER(vsmc/resample/residual            TRUE)
VSMC_HEADER(vsmc/resample/residual_stratified TRUE)
VSMC_HEADER(vsmc/resample/residual_systematic TRUE)
VSMC_HEADER(vsmc/resample/stratified          TRUE)
VSMC_HEADER(vsmc/resample/systematic          TRUE)

VSMC_HEADER(vsmc/rng/rng                       TRUE)
VSMC_HEADER(vsmc/rng/aes                       ${AESNI_FOUND})
VSMC_HEADER(vsmc/rng/ars                       ${AESNI_FOUND})
VSMC_HEADER(vsmc/rng/generator_wrapper         TRUE)
VSMC_HEADER(vsmc/rng/gsl_rng                   ${GSL_FOUND})
VSMC_HEADER(vsmc/rng/mkl_rng                   ${MKL_FOUND})
VSMC_HEADER(vsmc/rng/philox                    TRUE)
VSMC_HEADER(vsmc/rng/rdrand                    ${RDRAND_FOUND})
VSMC_HEADER(vsmc/rng/rng_set                   TRUE)
VSMC_HEADER(vsmc/rng/seed                      TRUE)
VSMC_HEADER(vsmc/rng/stable_distribution       TRUE)
VSMC_HEADER(vsmc/rng/threefry                  TRUE)
VSMC_HEADER(vsmc/rng/u01                       TRUE)
VSMC_HEADER(vsmc/rng/uniform_real_distribution TRUE)
VSMC_HEADER(vsmc/rng/xor_combine_engine        TRUE)
VSMC_HEADER(vsmc/rng/xorshift                  TRUE)

VSMC_HEADER(vsmc/smp/smp          TRUE)
VSMC_HEADER(vsmc/smp/adapter      TRUE)
VSMC_HEADER(vsmc/smp/backend_base TRUE)
VSMC_HEADER(vsmc/smp/backend_cilk ${CILK_FOUND})
VSMC_HEADER(vsmc/smp/backend_gcd  ${GCD_FOUND})
VSMC_HEADER(vsmc/smp/backend_omp  ${VSMC_OPENMP_FOUND})
VSMC_HEADER(vsmc/smp/backend_ppl  ${PPL_FOUND})
VSMC_HEADER(vsmc/smp/backend_seq  TRUE)
VSMC_HEADER(vsmc/smp/backend_std  ${CXX11LIB_THREAD_FOUND})
VSMC_HEADER(vsmc/smp/backend_tbb  ${TBB_FOUND})

VSMC_HEADER(vsmc/thread/thread              ${CXX11LIB_THREAD_FOUND})
VSMC_HEADER(vsmc/thread/blocked_range       TRUE)
VSMC_HEADER(vsmc/thread/parallel_accumulate ${CXX11LIB_THREAD_FOUND})
VSMC_HEADER(vsmc/thread/parallel_for        ${CXX11LIB_THREAD_FOUND})
VSMC_HEADER(vsmc/thread/parallel_reduce     ${CXX11LIB_THREAD_FOUND})
VSMC_HEADER(vsmc/thread/thread_guard        ${CXX11LIB_THREAD_FOUND})
VSMC_HEADER(vsmc/thread/thread_num          ${CXX11LIB_THREAD_FOUND})

VSMC_HEADER(vsmc/utility/utility        TRUE)
VSMC_HEADER(vsmc/utility/aligned_memory TRUE)
VSMC_HEADER(vsmc/utility/array          TRUE)
VSMC_HEADER(vsmc/utility/counter        TRUE)
VSMC_HEADER(vsmc/utility/cstring        TRUE)
VSMC_HEADER(vsmc/utility/cpuid          ${CPUID_FOUND})
VSMC_HEADER(vsmc/utility/hdf5io         ${HDF5_FOUND})
VSMC_HEADER(vsmc/utility/program_option TRUE)
VSMC_HEADER(vsmc/utility/progress       TRUE)
VSMC_HEADER(vsmc/utility/rdtsc          ${RDTSCP_FOUND})
VSMC_HEADER(vsmc/utility/stop_watch     TRUE)
