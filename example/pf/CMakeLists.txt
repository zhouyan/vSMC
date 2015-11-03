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

FUNCTION(ADD_PF_SMP_CHECK backend)
    ADD_CUSTOM_TARGET(pf_${backend}-check
        DEPENDS pf_${backend} pf-files
        COMMAND pf_${backend}
        "pf.data" "pf_${backend}" ">>pf.out"
        COMMENT "Running pf_${backend}"
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
    ADD_DEPENDENCIES(pf-check pf_${backend}-check)
ENDFUNCTION(ADD_PF_SMP_CHECK)

ADD_SMP_EXAMPLE(pf "HDF5")
COPY_FILE(pf pf.data)
COPY_FILE(pf pf.R)

ADD_CUSTOM_TARGET(pf-check)
ADD_DEPENDENCIES(check pf-check)

FOREACH (backend ${SMP_EXECUTABLES})
    ADD_PF_SMP_CHECK(${backend})
ENDFOREACH (backend ${SMP_EXECUTABLES})

ADD_CUSTOM_TARGET(pf-pdf
    DEPENDS pf-files pf-check
    COMMAND Rscript "${PROJECT_BINARY_DIR}/pf.R"
    COMMENT "Plotting pf-check results"
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
ADD_DEPENDENCIES(pdf pf-pdf)
