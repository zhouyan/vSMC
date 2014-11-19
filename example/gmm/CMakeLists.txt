# ============================================================================
#  vSMC/vSMCExample/gmm/CMakeLists.txt
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

PROJECT (vSMCExample-gmm CXX)
INCLUDE_DIRECTORIES (${PROJECT_BINARY_DIR}/include)
INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)

ADD_EXAMPLE (gmm smc; smc_pair; pmcmc; rjsmc; rjmcmc)

COPY_FILE (gmm gmm.data)

IF (OPENCL_FOUND AND RANDOM123_FOUND)
    COPY_FILE (gmm gmm_smc_cl.cl)
    ADD_EXECUTABLE (gmm_smc_cl ${PROJECT_SOURCE_DIR}/src/gmm_smc_cl.cpp)
    TARGET_LINK_LIBRARIES (gmm_smc_cl ${VSMC_LINK_LIBRARIES})
    TARGET_LINK_LIBRARIES (gmm_smc_cl ${OpenCL_LINK_LIBRARIES})
    ADD_DEPENDENCIES (gmm gmm_smc_cl)
    ADD_DEPENDENCIES (gmm_smc gmm_smc_cl)
    ADD_DEPENDENCIES (example_cl gmm_smc_cl)
ENDIF (OPENCL_FOUND AND RANDOM123_FOUND)
