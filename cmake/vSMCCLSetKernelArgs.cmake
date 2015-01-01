# ============================================================================
#  vSMC/cmake/vSMCCLSetKernelArgs.cmake
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

IF (NOT DEFINED VSMC_OPENCL_CL_SET_KERNEL_ARGS_MAX)
    SET (VSMC_OPENCL_CL_SET_KERNEL_ARGS_MAX 32)
ENDIF (NOT DEFINED VSMC_OPENCL_CL_SET_KERNEL_ARGS_MAX)

SET (ARG_MAX 0)
SET (TAB "        ")
SET (INT "    ")
WHILE (ARG_MAX LESS VSMC_OPENCL_CL_SET_KERNEL_ARGS_MAX)
    SET (FUNC "${FUNC}template <\n")
    SET (TARG 0)
    WHILE (TARG LESS ARG_MAX)
        SET (FUNC "${FUNC}${TAB}typename Arg${TARG},\n")
        MATH (EXPR TARG "${TARG} + 1")
    ENDWHILE (TARG LESS ARG_MAX)
    SET (FUNC "${FUNC}${TAB}typename Arg${ARG_MAX}>\n")

    SET (FUNC "${FUNC}inline void cl_set_kernel_args")
    SET (FUNC "${FUNC} (::cl::Kernel &kern, ::cl_uint offset,\n")
    SET (FARG 0)
    WHILE (FARG LESS ARG_MAX)
        SET (FUNC "${FUNC}${TAB}const Arg${FARG} &arg${FARG},\n")
        MATH (EXPR FARG "${FARG} + 1")
    ENDWHILE (FARG LESS ARG_MAX)
    SET (FUNC "${FUNC}${TAB}const Arg${ARG_MAX} &arg${ARG_MAX})\n{\n")

    SET (OFFSET 0)
    WHILE (OFFSET LESS ARG_MAX OR OFFSET EQUAL ARG_MAX)
        SET (FUNC "${FUNC}${INT}kern.setArg(")
        SET (FUNC "${FUNC}offset + ${OFFSET}, arg${OFFSET})\;\n")
        MATH (EXPR OFFSET "${OFFSET} + 1")
    ENDWHILE (OFFSET LESS ARG_MAX OR OFFSET EQUAL ARG_MAX)

    SET (FUNC "${FUNC}}\n\n")
    MATH (EXPR ARG_MAX "${ARG_MAX} + 1")
ENDWHILE (ARG_MAX LESS VSMC_OPENCL_CL_SET_KERNEL_ARGS_MAX)

SET (VSMC_OPENCL_CL_SET_KERNEL_ARGS ${FUNC})
CONFIGURE_FILE (
    ${PROJECT_SOURCE_DIR}/config/vsmc_opencl_internal_cl_set_kernel_args.hpp.in
    ${PROJECT_SOURCE_DIR}/include/vsmc/opencl/internal/cl_set_kernel_args.hpp)
