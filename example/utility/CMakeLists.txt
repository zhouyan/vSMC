# ============================================================================
#  vSMC/example/utility/CMakeLists.txt
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

PROJECT (vSMCExample-utility CXX)
INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)

ADD_CUSTOM_TARGET(utility)
ADD_DEPENDENCIES (example utility)

IF (CPUID_FOUND)
    ADD_EXECUTABLE (cpuid_info ${PROJECT_SOURCE_DIR}/src/cpuid_info.cpp)
    TARGET_LINK_LIBRARIES (cpuid_info ${VSMC_LINK_LIBRARIES})
    ADD_DEPENDENCIES (utility cpuid_info)
ENDIF (CPUID_FOUND)

ADD_EXECUTABLE (utility_cstring_memset
    ${PROJECT_SOURCE_DIR}/src/utility_cstring_memset.cpp)
ADD_EXECUTABLE (utility_cstring_memcpy
    ${PROJECT_SOURCE_DIR}/src/utility_cstring_memcpy.cpp)
ADD_EXECUTABLE (utility_cstring_memmove
    ${PROJECT_SOURCE_DIR}/src/utility_cstring_memmove.cpp)
TARGET_LINK_LIBRARIES (utility_cstring_memset  ${VSMC_LINK_LIBRARIES})
TARGET_LINK_LIBRARIES (utility_cstring_memcpy  ${VSMC_LINK_LIBRARIES})
TARGET_LINK_LIBRARIES (utility_cstring_memmove ${VSMC_LINK_LIBRARIES})
ADD_DEPENDENCIES (utility utility_cstring_memset)
ADD_DEPENDENCIES (utility utility_cstring_memcpy)
ADD_DEPENDENCIES (utility utility_cstring_memmove)
