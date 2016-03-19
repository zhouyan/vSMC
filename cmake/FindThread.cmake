# ============================================================================
#  vSMC/cmake/FindThread.cmake
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

# Find thread library
#
# The following variable is set
#
# THREAD_FOUND          - TRUE if threads are found
# Thread_LINK_LIBRARIES - Set to CMAKE_THREAD_LIBS_INIT and caches

IF(DEFINED THREAD_FOUND)
    RETURN()
ENDIF(DEFINED THREAD_FOUND)

INCLUDE(FindThreads)
IF(CMAKE_THREAD_LIBS_INIT)
    SET(THREAD_FOUND TRUE CACHE BOOL "Threads found")
    SET(Thread_LINK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} CACHE STRING
        "Thread link libraries")
ELSE(CMAKE_THREAD_LIBS_INIT)
    SET(THREAD_FOUND FALSE CACHE BOOL "Threads found")
ENDIF(CMAKE_THREAD_LIBS_INIT)
