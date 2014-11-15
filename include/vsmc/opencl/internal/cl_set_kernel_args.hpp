//============================================================================
// vSMC/config/vsmc_opencl_internal_cl_set_kernel_args.hpp.in
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

template <
        typename Arg0>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0)
{
    kern.setArg(offset + 0, arg0);
}

template <
        typename Arg0,
        typename Arg1>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7,
        typename Arg8>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7,
        const Arg8 &arg8)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
    kern.setArg(offset + 8, arg8);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7,
        typename Arg8,
        typename Arg9>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7,
        const Arg8 &arg8,
        const Arg9 &arg9)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
    kern.setArg(offset + 8, arg8);
    kern.setArg(offset + 9, arg9);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7,
        typename Arg8,
        typename Arg9,
        typename Arg10>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7,
        const Arg8 &arg8,
        const Arg9 &arg9,
        const Arg10 &arg10)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
    kern.setArg(offset + 8, arg8);
    kern.setArg(offset + 9, arg9);
    kern.setArg(offset + 10, arg10);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7,
        typename Arg8,
        typename Arg9,
        typename Arg10,
        typename Arg11>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7,
        const Arg8 &arg8,
        const Arg9 &arg9,
        const Arg10 &arg10,
        const Arg11 &arg11)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
    kern.setArg(offset + 8, arg8);
    kern.setArg(offset + 9, arg9);
    kern.setArg(offset + 10, arg10);
    kern.setArg(offset + 11, arg11);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7,
        typename Arg8,
        typename Arg9,
        typename Arg10,
        typename Arg11,
        typename Arg12>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7,
        const Arg8 &arg8,
        const Arg9 &arg9,
        const Arg10 &arg10,
        const Arg11 &arg11,
        const Arg12 &arg12)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
    kern.setArg(offset + 8, arg8);
    kern.setArg(offset + 9, arg9);
    kern.setArg(offset + 10, arg10);
    kern.setArg(offset + 11, arg11);
    kern.setArg(offset + 12, arg12);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7,
        typename Arg8,
        typename Arg9,
        typename Arg10,
        typename Arg11,
        typename Arg12,
        typename Arg13>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7,
        const Arg8 &arg8,
        const Arg9 &arg9,
        const Arg10 &arg10,
        const Arg11 &arg11,
        const Arg12 &arg12,
        const Arg13 &arg13)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
    kern.setArg(offset + 8, arg8);
    kern.setArg(offset + 9, arg9);
    kern.setArg(offset + 10, arg10);
    kern.setArg(offset + 11, arg11);
    kern.setArg(offset + 12, arg12);
    kern.setArg(offset + 13, arg13);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7,
        typename Arg8,
        typename Arg9,
        typename Arg10,
        typename Arg11,
        typename Arg12,
        typename Arg13,
        typename Arg14>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7,
        const Arg8 &arg8,
        const Arg9 &arg9,
        const Arg10 &arg10,
        const Arg11 &arg11,
        const Arg12 &arg12,
        const Arg13 &arg13,
        const Arg14 &arg14)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
    kern.setArg(offset + 8, arg8);
    kern.setArg(offset + 9, arg9);
    kern.setArg(offset + 10, arg10);
    kern.setArg(offset + 11, arg11);
    kern.setArg(offset + 12, arg12);
    kern.setArg(offset + 13, arg13);
    kern.setArg(offset + 14, arg14);
}

template <
        typename Arg0,
        typename Arg1,
        typename Arg2,
        typename Arg3,
        typename Arg4,
        typename Arg5,
        typename Arg6,
        typename Arg7,
        typename Arg8,
        typename Arg9,
        typename Arg10,
        typename Arg11,
        typename Arg12,
        typename Arg13,
        typename Arg14,
        typename Arg15>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg0 &arg0,
        const Arg1 &arg1,
        const Arg2 &arg2,
        const Arg3 &arg3,
        const Arg4 &arg4,
        const Arg5 &arg5,
        const Arg6 &arg6,
        const Arg7 &arg7,
        const Arg8 &arg8,
        const Arg9 &arg9,
        const Arg10 &arg10,
        const Arg11 &arg11,
        const Arg12 &arg12,
        const Arg13 &arg13,
        const Arg14 &arg14,
        const Arg15 &arg15)
{
    kern.setArg(offset + 0, arg0);
    kern.setArg(offset + 1, arg1);
    kern.setArg(offset + 2, arg2);
    kern.setArg(offset + 3, arg3);
    kern.setArg(offset + 4, arg4);
    kern.setArg(offset + 5, arg5);
    kern.setArg(offset + 6, arg6);
    kern.setArg(offset + 7, arg7);
    kern.setArg(offset + 8, arg8);
    kern.setArg(offset + 9, arg9);
    kern.setArg(offset + 10, arg10);
    kern.setArg(offset + 11, arg11);
    kern.setArg(offset + 12, arg12);
    kern.setArg(offset + 13, arg13);
    kern.setArg(offset + 14, arg14);
    kern.setArg(offset + 15, arg15);
}


