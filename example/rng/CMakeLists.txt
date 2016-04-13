# ============================================================================
#  vSMC/example/rng/CMakeLists.txt
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

PROJECT(vSMCExample-rng CXX)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_CUSTOM_TARGET(rng)
ADD_DEPENDENCIES(example rng)

ADD_CUSTOM_TARGET(rng-files)
ADD_DEPENDENCIES(rng rng-files)
ADD_DEPENDENCIES(example-files rng-files)

FUNCTION(VSMC_ADD_RNG_TEST name)
    VSMC_ADD_EXECUTABLE(rng_${name} ${PROJECT_SOURCE_DIR}/src/rng_${name}.cpp)
    ADD_DEPENDENCIES(rng rng_${name})
ENDFUNCTION(VSMC_ADD_RNG_TEST)

VSMC_ADD_RNG_TEST(engine)
VSMC_ADD_RNG_TEST(u01)
VSMC_ADD_RNG_TEST(u01_sequence)
VSMC_ADD_RNG_TEST(uniform_bits)

IF(MKL_FOUND)
    VSMC_ADD_RNG_TEST(mkl_brng)
ENDIF(MKL_FOUND)

IF(Boost_FOUND)
    VSMC_ADD_RNG_TEST(distribution)
ENDIF(Boost_FOUND)

VSMC_ADD_FILE(rng rng_engine_kat_Philox2x32.txt)
VSMC_ADD_FILE(rng rng_engine_kat_Philox2x64.txt)
VSMC_ADD_FILE(rng rng_engine_kat_Philox4x32.txt)
VSMC_ADD_FILE(rng rng_engine_kat_Philox4x64.txt)

VSMC_ADD_FILE(rng rng_engine_kat_Threefry2x32.txt)
VSMC_ADD_FILE(rng rng_engine_kat_Threefry2x64.txt)
VSMC_ADD_FILE(rng rng_engine_kat_Threefry4x32.txt)
VSMC_ADD_FILE(rng rng_engine_kat_Threefry4x64.txt)
VSMC_ADD_FILE(rng rng_engine_kat_Threefry8x64.txt)
VSMC_ADD_FILE(rng rng_engine_kat_Threefry16x64.txt)

IF(AESNI_FOUND)
    VSMC_ADD_RNG_TEST(aes)
    VSMC_ADD_FILE(rng rng_engine_kat_AES128.txt)
    VSMC_ADD_FILE(rng rng_engine_kat_AES192.txt)
    VSMC_ADD_FILE(rng rng_engine_kat_AES256.txt)
    VSMC_ADD_FILE(rng rng_engine_kat_ARS.txt)
ENDIF(AESNI_FOUND)
