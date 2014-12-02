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
        typename Arg15,
        typename Arg16>
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
        const Arg15 &arg15,
        const Arg16 &arg16)
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
    kern.setArg(offset + 16, arg16);
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
        typename Arg15,
        typename Arg16,
        typename Arg17>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23,
        typename Arg24>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23,
        const Arg24 &arg24)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
    kern.setArg(offset + 24, arg24);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23,
        typename Arg24,
        typename Arg25>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23,
        const Arg24 &arg24,
        const Arg25 &arg25)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
    kern.setArg(offset + 24, arg24);
    kern.setArg(offset + 25, arg25);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23,
        typename Arg24,
        typename Arg25,
        typename Arg26>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23,
        const Arg24 &arg24,
        const Arg25 &arg25,
        const Arg26 &arg26)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
    kern.setArg(offset + 24, arg24);
    kern.setArg(offset + 25, arg25);
    kern.setArg(offset + 26, arg26);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23,
        typename Arg24,
        typename Arg25,
        typename Arg26,
        typename Arg27>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23,
        const Arg24 &arg24,
        const Arg25 &arg25,
        const Arg26 &arg26,
        const Arg27 &arg27)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
    kern.setArg(offset + 24, arg24);
    kern.setArg(offset + 25, arg25);
    kern.setArg(offset + 26, arg26);
    kern.setArg(offset + 27, arg27);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23,
        typename Arg24,
        typename Arg25,
        typename Arg26,
        typename Arg27,
        typename Arg28>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23,
        const Arg24 &arg24,
        const Arg25 &arg25,
        const Arg26 &arg26,
        const Arg27 &arg27,
        const Arg28 &arg28)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
    kern.setArg(offset + 24, arg24);
    kern.setArg(offset + 25, arg25);
    kern.setArg(offset + 26, arg26);
    kern.setArg(offset + 27, arg27);
    kern.setArg(offset + 28, arg28);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23,
        typename Arg24,
        typename Arg25,
        typename Arg26,
        typename Arg27,
        typename Arg28,
        typename Arg29>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23,
        const Arg24 &arg24,
        const Arg25 &arg25,
        const Arg26 &arg26,
        const Arg27 &arg27,
        const Arg28 &arg28,
        const Arg29 &arg29)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
    kern.setArg(offset + 24, arg24);
    kern.setArg(offset + 25, arg25);
    kern.setArg(offset + 26, arg26);
    kern.setArg(offset + 27, arg27);
    kern.setArg(offset + 28, arg28);
    kern.setArg(offset + 29, arg29);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23,
        typename Arg24,
        typename Arg25,
        typename Arg26,
        typename Arg27,
        typename Arg28,
        typename Arg29,
        typename Arg30>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23,
        const Arg24 &arg24,
        const Arg25 &arg25,
        const Arg26 &arg26,
        const Arg27 &arg27,
        const Arg28 &arg28,
        const Arg29 &arg29,
        const Arg30 &arg30)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
    kern.setArg(offset + 24, arg24);
    kern.setArg(offset + 25, arg25);
    kern.setArg(offset + 26, arg26);
    kern.setArg(offset + 27, arg27);
    kern.setArg(offset + 28, arg28);
    kern.setArg(offset + 29, arg29);
    kern.setArg(offset + 30, arg30);
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
        typename Arg15,
        typename Arg16,
        typename Arg17,
        typename Arg18,
        typename Arg19,
        typename Arg20,
        typename Arg21,
        typename Arg22,
        typename Arg23,
        typename Arg24,
        typename Arg25,
        typename Arg26,
        typename Arg27,
        typename Arg28,
        typename Arg29,
        typename Arg30,
        typename Arg31>
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
        const Arg15 &arg15,
        const Arg16 &arg16,
        const Arg17 &arg17,
        const Arg18 &arg18,
        const Arg19 &arg19,
        const Arg20 &arg20,
        const Arg21 &arg21,
        const Arg22 &arg22,
        const Arg23 &arg23,
        const Arg24 &arg24,
        const Arg25 &arg25,
        const Arg26 &arg26,
        const Arg27 &arg27,
        const Arg28 &arg28,
        const Arg29 &arg29,
        const Arg30 &arg30,
        const Arg31 &arg31)
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
    kern.setArg(offset + 16, arg16);
    kern.setArg(offset + 17, arg17);
    kern.setArg(offset + 18, arg18);
    kern.setArg(offset + 19, arg19);
    kern.setArg(offset + 20, arg20);
    kern.setArg(offset + 21, arg21);
    kern.setArg(offset + 22, arg22);
    kern.setArg(offset + 23, arg23);
    kern.setArg(offset + 24, arg24);
    kern.setArg(offset + 25, arg25);
    kern.setArg(offset + 26, arg26);
    kern.setArg(offset + 27, arg27);
    kern.setArg(offset + 28, arg28);
    kern.setArg(offset + 29, arg29);
    kern.setArg(offset + 30, arg30);
    kern.setArg(offset + 31, arg31);
}


