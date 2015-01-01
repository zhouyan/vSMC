# ============================================================================
#  vSMC/example/pet/CMakeLists.txt
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

PROJECT (vSMCExample-pet CXX)
INCLUDE_DIRECTORIES (${PROJECT_BINARY_DIR}/include)
INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)

SET (PET_ALGS smc; pmcmc; is)
ADD_SMP_EXAMPLE (pet "${PET_ALGS}")

COPY_FILE (pet pet_simulate.data)
COPY_FILE (pet pet_time.data)
COPY_FILE (pet pet_prior_i.data)
COPY_FILE (pet pet_prior_n.data)
COPY_FILE (pet pet_proposal_i.data)
COPY_FILE (pet pet_proposal_n.data)
COPY_FILE (pet pet_simulate_data.config)
COPY_FILE (pet pet_real_data.config)

COPY_FILE_OPTIONAL (pet pet_conv.data)
COPY_FILE_OPTIONAL (pet pet_real.data)
IF (NOT EXISTS ${PROJECT_SOURCE_DIR}/pet_conv.data)
    MESSAGE (STATUS "Example PET: pet_conv.data NOT Found. Use pet_conv.R to generate it (take a long time!)")
ENDIF (NOT EXISTS ${PROJECT_SOURCE_DIR}/pet_conv.data)
IF (NOT EXISTS ${PROJECT_SOURCE_DIR}/pet_real.data)
    MESSAGE (STATUS "Example PET: real data not available")
ENDIF (NOT EXISTS ${PROJECT_SOURCE_DIR}/pet_real.data)
