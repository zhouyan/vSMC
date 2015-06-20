# ============================================================================
#  vSMC/example/CMakeLists.txt
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
# Copyright (c) 2013-2015, Yan Zhou
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ============================================================================

PROJECT (vSMCExample C CXX)

INCLUDE (vSMCExampleFunctions)
ADD_DEFINITIONS (${EXAMPLE_DEFINITIONS})

##############################################################################
# Check examples
##############################################################################

ADD_CUSTOM_TARGET (example-files)
ADD_DEPENDENCIES (example example-files)

SET (EXAMPLES ${EXAMPLES} "vsmc")
ADD_SUBDIRECTORY (vsmc)

SET (EXAMPLES ${EXAMPLES} "gmm")
ADD_SUBDIRECTORY (gmm)

SET (EXAMPLES ${EXAMPLES} "pf")
ADD_SUBDIRECTORY (pf)

SET (EXAMPLES ${EXAMPLES} "rng")
ADD_SUBDIRECTORY (rng)

##############################################################################
# Enable examples
##############################################################################

MESSAGE (STATUS "=================== Enable examples ===================")
FOREACH (example ${EXAMPLES})
    MESSAGE (STATUS ${example})
ENDFOREACH (example ${EXAMPLES})

MESSAGE (STATUS "=======================================================")
