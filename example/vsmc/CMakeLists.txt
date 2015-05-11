# ============================================================================
#  vSMC/example/vsmc/CMakeLists.txt
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

PROJECT (vSMCExample-vsmc CXX)

ADD_CUSTOM_TARGET (vsmc)
ADD_DEPENDENCIES (example vsmc)

ADD_HEADER_EXECUTABLE(vsmc/core/core TRUE)
ADD_HEADER_EXECUTABLE(vsmc/core/monitor         TRUE)
ADD_HEADER_EXECUTABLE(vsmc/core/particle        TRUE)
ADD_HEADER_EXECUTABLE(vsmc/core/path            TRUE)
ADD_HEADER_EXECUTABLE(vsmc/core/sampler         TRUE)
ADD_HEADER_EXECUTABLE(vsmc/core/single_particle TRUE)
ADD_HEADER_EXECUTABLE(vsmc/core/state_matrix    TRUE)
ADD_HEADER_EXECUTABLE(vsmc/core/state_tuple     TRUE)
ADD_HEADER_EXECUTABLE(vsmc/core/weight_set      TRUE)

ADD_HEADER_EXECUTABLE(vsmc/internal/assert   TRUE)
ADD_HEADER_EXECUTABLE(vsmc/internal/common   TRUE)
ADD_HEADER_EXECUTABLE(vsmc/internal/compiler TRUE)
ADD_HEADER_EXECUTABLE(vsmc/internal/config   TRUE)
ADD_HEADER_EXECUTABLE(vsmc/internal/defines  TRUE)
ADD_HEADER_EXECUTABLE(vsmc/internal/forward  TRUE)
ADD_HEADER_EXECUTABLE(vsmc/internal/traits   TRUE)

ADD_HEADER_EXECUTABLE(vsmc/math/math TRUE)
ADD_HEADER_EXECUTABLE(vsmc/math/constants TRUE)
ADD_HEADER_EXECUTABLE(vsmc/math/cblas     TRUE)
ADD_HEADER_EXECUTABLE(vsmc/math/vmath     TRUE)

ADD_HEADER_EXECUTABLE(vsmc/mpi/mpi ${VSMC_MPI_FOUND} "MPI")
ADD_HEADER_EXECUTABLE(vsmc/mpi/backend_mpi ${VSMC_MPI_FOUND} "MPI")
ADD_HEADER_EXECUTABLE(vsmc/mpi/mpi_manager ${VSMC_MPI_FOUND} "MPI")

ADD_HEADER_EXECUTABLE(vsmc/opencl/opencl ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/backend_cl          ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/cl_buffer           ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/cl_manager          ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/cl_manip            ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/cl_query            ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/cl_setup            ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/cl_type             ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/internal/cl_copy    ${OPENCL_FOUND} "OCL")
ADD_HEADER_EXECUTABLE(vsmc/opencl/internal/common     ${OPENCL_FOUND} "OCL")

ADD_HEADER_EXECUTABLE(vsmc/resample/resample            TRUE)
ADD_HEADER_EXECUTABLE(vsmc/resample/multinomial         TRUE)
ADD_HEADER_EXECUTABLE(vsmc/resample/residual            TRUE)
ADD_HEADER_EXECUTABLE(vsmc/resample/residual_stratified TRUE)
ADD_HEADER_EXECUTABLE(vsmc/resample/residual_systematic TRUE)
ADD_HEADER_EXECUTABLE(vsmc/resample/stratified          TRUE)
ADD_HEADER_EXECUTABLE(vsmc/resample/systematic          TRUE)
ADD_HEADER_EXECUTABLE(vsmc/resample/internal/common     TRUE)

ADD_HEADER_EXECUTABLE(vsmc/rng/rng TRUE "MKL")
ADD_HEADER_EXECUTABLE(vsmc/rng/aes                       ${AESNI_FOUND})
ADD_HEADER_EXECUTABLE(vsmc/rng/ars                       ${AESNI_FOUND})
ADD_HEADER_EXECUTABLE(vsmc/rng/discrete_distribution     TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/mkl                       ${MKL_FOUND} "MKL")
ADD_HEADER_EXECUTABLE(vsmc/rng/philox                    TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/rdrand                    ${RDRAND_FOUND})
ADD_HEADER_EXECUTABLE(vsmc/rng/rng_set                   TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/seed                      TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/stable_distribution       TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/threefry                  TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/threefry_avx2             ${AVX2_FOUND})
ADD_HEADER_EXECUTABLE(vsmc/rng/threefry_sse2             ${SSE2_FOUND})
ADD_HEADER_EXECUTABLE(vsmc/rng/u01                       TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/uniform_real_distribution TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/xor_combine_engine        TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/xorshift                  TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/internal/common           TRUE)
ADD_HEADER_EXECUTABLE(vsmc/rng/internal/m128i            ${SSE2_FOUND})
ADD_HEADER_EXECUTABLE(vsmc/rng/internal/m256i            ${AVX2_FOUND})

ADD_HEADER_EXECUTABLE(vsmc/smp/smp TRUE "CILK" "GCD" "OMP" "STD" "TBB")
ADD_HEADER_EXECUTABLE(vsmc/smp/backend_base TRUE)
ADD_HEADER_EXECUTABLE(vsmc/smp/backend_omp  ${VSMC_OPENMP_FOUND} "OMP")
ADD_HEADER_EXECUTABLE(vsmc/smp/backend_seq  TRUE)
ADD_HEADER_EXECUTABLE(vsmc/smp/backend_tbb  ${TBB_FOUND} "TBB")

ADD_HEADER_EXECUTABLE(vsmc/utility/utility TRUE "HDF5")
ADD_HEADER_EXECUTABLE(vsmc/utility/aligned_memory TRUE)
ADD_HEADER_EXECUTABLE(vsmc/utility/cpuid          ${CPUID_FOUND})
ADD_HEADER_EXECUTABLE(vsmc/utility/hdf5io         ${HDF5_FOUND} "HDF5")
ADD_HEADER_EXECUTABLE(vsmc/utility/program_option TRUE)
ADD_HEADER_EXECUTABLE(vsmc/utility/mkl            ${MKL_FOUND} "MKL")
ADD_HEADER_EXECUTABLE(vsmc/utility/progress       TRUE)
ADD_HEADER_EXECUTABLE(vsmc/utility/rdtsc          ${RDTSCP_FOUND})
ADD_HEADER_EXECUTABLE(vsmc/utility/stop_watch     TRUE)
