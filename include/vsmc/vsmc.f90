! ============================================================================
!  vSMC/include/vsmc/vsmc.hpp
! ----------------------------------------------------------------------------
!                          vSMC: Scalable Monte Carlo
! ----------------------------------------------------------------------------
!  Copyright (c) 2013-2015, Yan Zhou
!  All rights reserved.
!
!  Redistribution and use in source and binary forms, with or without
!  modification, are permitted provided that the following conditions are met:
!
!    Redistributions of source code must retain the above copyright notice,
!    this list of conditions and the following disclaimer.
!
!    Redistributions in binary form must reproduce the above copyright notice,
!    this list of conditions and the following disclaimer in the documentation
!    and/or other materials provided with the distribution.
!
!  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
!  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
!  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
!  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
!  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
!  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
!  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
!  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
!  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
!  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
!  POSSIBILITY OF SUCH DAMAGE.
! ============================================================================

module vsmc
    use, intrinsic :: iso_c_binding
    implicit none

    type, bind(c) :: vsmc_rng
        integer(kind = c_int), dimension(64) :: state
    end type vsmc_rng

    interface
        function vsmc_rng_size() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_rng_size
        end function vsmc_rng_size
    end interface

    interface
        subroutine vsmc_rng_init(rng, seed) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: seed
        end subroutine vsmc_rng_init
    end interface

    interface
        subroutine vsmc_rng_seed(rng, seed) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: seed
        end subroutine vsmc_rng_seed
    end interface

    interface
        subroutine vsmc_rng_get_key(rng, n, key) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: key
        end subroutine vsmc_rng_get_key
    end interface

    interface
        subroutine vsmc_rng_set_key(rng, n, key) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: key
        end subroutine vsmc_rng_set_key
    end interface

    interface
        subroutine vsmc_rng_get_ctr(rng, n, ctr) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: ctr
        end subroutine vsmc_rng_get_ctr
    end interface

    interface
        subroutine vsmc_rng_set_ctr(rng, n, ctr) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: ctr
        end subroutine vsmc_rng_set_ctr
    end interface

    interface
        subroutine vsmc_rng_rand(rng, n, r) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: r
        end subroutine vsmc_rng_rand
    end interface

    interface
        subroutine vsmc_rng_uniform_int(rng, n, r, a, b) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: r
            integer(kind = c_int), value :: a
            integer(kind = c_int), value :: b
        end subroutine vsmc_rng_uniform_int
    end interface

    interface
        subroutine vsmc_rng_uniform_real(rng, n, r, a, b) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: a
            real(kind = c_double), value :: b
        end subroutine vsmc_rng_uniform_real
    end interface

    interface
        subroutine vsmc_rng_uniform_real_cc(rng, n, r, a, b) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: a
            real(kind = c_double), value :: b
        end subroutine vsmc_rng_uniform_real_cc
    end interface

    interface
        subroutine vsmc_rng_uniform_real_co(rng, n, r, a, b) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: a
            real(kind = c_double), value :: b
        end subroutine vsmc_rng_uniform_real_co
    end interface

    interface
        subroutine vsmc_rng_uniform_real_oc(rng, n, r, a, b) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: a
            real(kind = c_double), value :: b
        end subroutine vsmc_rng_uniform_real_oc
    end interface

    interface
        subroutine vsmc_rng_uniform_real_oo(rng, n, r, a, b) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: a
            real(kind = c_double), value :: b
        end subroutine vsmc_rng_uniform_real_oo
    end interface

    interface
        subroutine vsmc_rng_bernoulli(rng, n, r, p) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: r
            real(kind = c_double), value :: p
        end subroutine vsmc_rng_bernoulli
    end interface

    interface
        subroutine vsmc_rng_binomial(rng, n, r, t, p) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: r
            integer(kind = c_int), value :: t
            real(kind = c_double), value :: p
        end subroutine vsmc_rng_binomial
    end interface

    interface
        subroutine vsmc_rng_negative_binomial(rng, n, r, k, p) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: r
            integer(kind = c_int), value :: k
            real(kind = c_double), value :: p
        end subroutine vsmc_rng_negative_binomial
    end interface

    interface
        subroutine vsmc_rng_geometric(rng, n, r, p) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: r
            real(kind = c_double), value :: p
        end subroutine vsmc_rng_geometric
    end interface

    interface
        subroutine vsmc_rng_poisson(rng, n, r, mean) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: r
            real(kind = c_double), value :: mean
        end subroutine vsmc_rng_poisson
    end interface

    interface
        subroutine vsmc_rng_exponential(rng, n, r, lambda) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: rate
        end subroutine vsmc_rng_exponential
    end interface

    interface
        subroutine vsmc_rng_gamma(rng, n, r, shap, scal) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: shap
            real(kind = c_double), value :: scal
        end subroutine vsmc_rng_gamma
    end interface

    interface
        subroutine vsmc_rng_weibull(rng, n, r, shap, scal) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: shap
            real(kind = c_double), value :: scal
        end subroutine vsmc_rng_weibull
    end interface

    interface
        subroutine vsmc_rng_extreme_value(rng, n, r, location, scal) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: location
            real(kind = c_double), value :: scal
        end subroutine vsmc_rng_extreme_value
    end interface

    interface
        subroutine vsmc_rng_normal(rng, n, r, mean, stddev) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: mean
            real(kind = c_double), value :: stddev
        end subroutine vsmc_rng_normal
    end interface

    interface
        subroutine vsmc_rng_lognormal(rng, n, r, m, s) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: m
            real(kind = c_double), value :: s
        end subroutine vsmc_rng_lognormal
    end interface

    interface
        subroutine vsmc_rng_chi_squared(rng, n, r, df) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: df
        end subroutine vsmc_rng_chi_squared
    end interface

    interface
        subroutine vsmc_rng_cauchy(rng, n, r, a, b) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: a
            real(kind = c_double), value :: b
        end subroutine vsmc_rng_cauchy
    end interface

    interface
        subroutine vsmc_rng_fisher_f(rng, n, r, df1, df2) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: df1
            real(kind = c_double), value :: df2
        end subroutine vsmc_rng_fisher_f
    end interface

    interface
        subroutine vsmc_rng_student_t(rng, n, r, df) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: df
        end subroutine vsmc_rng_student_t
    end interface

    interface
        subroutine vsmc_rng_laplace(rng, n, r, location, scal) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: location
            real(kind = c_double), value :: scal
        end subroutine vsmc_rng_laplace
    end interface

    interface
        subroutine vsmc_rng_stable(&
                rng, n, r, stability, skewness, location, scal) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: stability
            real(kind = c_double), value :: skewness
            real(kind = c_double), value :: location
            real(kind = c_double), value :: scal
        end subroutine vsmc_rng_stable
    end interface

    interface
        subroutine vsmc_rng_discrete(rng, n, r, m, weight, normalized) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: r
            integer(kind = c_int), value :: m
            real(kind = c_double), dimension(*) :: weight
            integer(kind = c_int), value :: normalized
        end subroutine vsmc_rng_discrete
    end interface

    interface
        subroutine vsmc_rng_u01_sorted(n, u01, u01seq)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: u01
            real(kind = c_double), dimension(*) :: u01seq
        end subroutine vsmc_rng_u01_sorted
    end interface

    interface
        subroutine vsmc_rng_u01_stratified(n, u01, u01seq)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: u01
            real(kind = c_double), dimension(*) :: u01seq
        end subroutine vsmc_rng_u01_stratified
    end interface

    interface
        subroutine vsmc_rng_u01_systematic(n, u01, u01seq)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int), value :: n
            real(kind = c_double), value :: u01
            real(kind = c_double), dimension(*) :: u01seq
        end subroutine vsmc_rng_u01_systematic
    end interface

    interface
        function vsmc_mkl_brng_mt19937() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_mt19937
        end function vsmc_mkl_brng_mt19937
    end interface

    interface
        function vsmc_mkl_brng_minstd_rand0() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_minstd_rand0
        end function vsmc_mkl_brng_minstd_rand0
    end interface

    interface
        function vsmc_mkl_brng_minstd_rand() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_minstd_rand
        end function vsmc_mkl_brng_minstd_rand
    end interface

    interface
        function vsmc_mkl_brng_mt19937_64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_mt19937_64
        end function vsmc_mkl_brng_mt19937_64
    end interface

    interface
        function vsmc_mkl_brng_ranlux24_base() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ranlux24_base
        end function vsmc_mkl_brng_ranlux24_base
    end interface

    interface
        function vsmc_mkl_brng_ranlux48_base() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ranlux48_base
        end function vsmc_mkl_brng_ranlux48_base
    end interface

    interface
        function vsmc_mkl_brng_ranlux24() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ranlux24
        end function vsmc_mkl_brng_ranlux24
    end interface

    interface
        function vsmc_mkl_brng_ranlux48() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ranlux48
        end function vsmc_mkl_brng_ranlux48
    end interface

    interface
        function vsmc_mkl_brng_knuth_b() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_knuth_b
        end function vsmc_mkl_brng_knuth_b
    end interface

    interface
        function vsmc_mkl_brng_xorshift1x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift1x32
        end function vsmc_mkl_brng_xorshift1x32
    end interface

    interface
        function vsmc_mkl_brng_xorshift2x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift2x32
        end function vsmc_mkl_brng_xorshift2x32
    end interface

    interface
        function vsmc_mkl_brng_xorshift4x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift4x32
        end function vsmc_mkl_brng_xorshift4x32
    end interface

    interface
        function vsmc_mkl_brng_xorshift8x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift8x32
        end function vsmc_mkl_brng_xorshift8x32
    end interface

    interface
        function vsmc_mkl_brng_xorshift16x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift16x32
        end function vsmc_mkl_brng_xorshift16x32
    end interface

    interface
        function vsmc_mkl_brng_xorshift32x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift32x32
        end function vsmc_mkl_brng_xorshift32x32
    end interface

    interface
        function vsmc_mkl_brng_xorshift64x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift64x32
        end function vsmc_mkl_brng_xorshift64x32
    end interface

    interface
        function vsmc_mkl_brng_xorshift128x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift128x32
        end function vsmc_mkl_brng_xorshift128x32
    end interface

    interface
        function vsmc_mkl_brng_xorshift1x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift1x64
        end function vsmc_mkl_brng_xorshift1x64
    end interface

    interface
        function vsmc_mkl_brng_xorshift2x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift2x64
        end function vsmc_mkl_brng_xorshift2x64
    end interface

    interface
        function vsmc_mkl_brng_xorshift4x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift4x64
        end function vsmc_mkl_brng_xorshift4x64
    end interface

    interface
        function vsmc_mkl_brng_xorshift8x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift8x64
        end function vsmc_mkl_brng_xorshift8x64
    end interface

    interface
        function vsmc_mkl_brng_xorshift16x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift16x64
        end function vsmc_mkl_brng_xorshift16x64
    end interface

    interface
        function vsmc_mkl_brng_xorshift32x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift32x64
        end function vsmc_mkl_brng_xorshift32x64
    end interface

    interface
        function vsmc_mkl_brng_xorshift64x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorshift64x64
        end function vsmc_mkl_brng_xorshift64x64
    end interface

    interface
        function vsmc_mkl_brng_xorwow1x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow1x32
        end function vsmc_mkl_brng_xorwow1x32
    end interface

    interface
        function vsmc_mkl_brng_xorwow2x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow2x32
        end function vsmc_mkl_brng_xorwow2x32
    end interface

    interface
        function vsmc_mkl_brng_xorwow4x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow4x32
        end function vsmc_mkl_brng_xorwow4x32
    end interface

    interface
        function vsmc_mkl_brng_xorwow8x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow8x32
        end function vsmc_mkl_brng_xorwow8x32
    end interface

    interface
        function vsmc_mkl_brng_xorwow16x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow16x32
        end function vsmc_mkl_brng_xorwow16x32
    end interface

    interface
        function vsmc_mkl_brng_xorwow32x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow32x32
        end function vsmc_mkl_brng_xorwow32x32
    end interface

    interface
        function vsmc_mkl_brng_xorwow64x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow64x32
        end function vsmc_mkl_brng_xorwow64x32
    end interface

    interface
        function vsmc_mkl_brng_xorwow128x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow128x32
        end function vsmc_mkl_brng_xorwow128x32
    end interface

    interface
        function vsmc_mkl_brng_xorwow1x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow1x64
        end function vsmc_mkl_brng_xorwow1x64
    end interface

    interface
        function vsmc_mkl_brng_xorwow2x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow2x64
        end function vsmc_mkl_brng_xorwow2x64
    end interface

    interface
        function vsmc_mkl_brng_xorwow4x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow4x64
        end function vsmc_mkl_brng_xorwow4x64
    end interface

    interface
        function vsmc_mkl_brng_xorwow8x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow8x64
        end function vsmc_mkl_brng_xorwow8x64
    end interface

    interface
        function vsmc_mkl_brng_xorwow16x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow16x64
        end function vsmc_mkl_brng_xorwow16x64
    end interface

    interface
        function vsmc_mkl_brng_xorwow32x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow32x64
        end function vsmc_mkl_brng_xorwow32x64
    end interface

    interface
        function vsmc_mkl_brng_xorwow64x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_xorwow64x64
        end function vsmc_mkl_brng_xorwow64x64
    end interface

    interface
        function vsmc_mkl_brng_philox2x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_philox2x32
        end function vsmc_mkl_brng_philox2x32
    end interface

    interface
        function vsmc_mkl_brng_philox4x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_philox4x32
        end function vsmc_mkl_brng_philox4x32
    end interface

    interface
        function vsmc_mkl_brng_philox2x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_philox2x64
        end function vsmc_mkl_brng_philox2x64
    end interface

    interface
        function vsmc_mkl_brng_philox4x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_philox4x64
        end function vsmc_mkl_brng_philox4x64
    end interface

    interface
        function vsmc_mkl_brng_threefry2x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry2x32
        end function vsmc_mkl_brng_threefry2x32
    end interface

    interface
        function vsmc_mkl_brng_threefry4x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry4x32
        end function vsmc_mkl_brng_threefry4x32
    end interface

    interface
        function vsmc_mkl_brng_threefry2x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry2x64
        end function vsmc_mkl_brng_threefry2x64
    end interface

    interface
        function vsmc_mkl_brng_threefry4x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry4x64
        end function vsmc_mkl_brng_threefry4x64
    end interface

    interface
        function vsmc_mkl_brng_threefry2x32avx2() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry2x32avx2
        end function vsmc_mkl_brng_threefry2x32avx2
    end interface

    interface
        function vsmc_mkl_brng_threefry4x32avx2() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry4x32avx2
        end function vsmc_mkl_brng_threefry4x32avx2
    end interface

    interface
        function vsmc_mkl_brng_threefry2x64avx2() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry2x64avx2
        end function vsmc_mkl_brng_threefry2x64avx2
    end interface

    interface
        function vsmc_mkl_brng_threefry4x64avx2() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry4x64avx2
        end function vsmc_mkl_brng_threefry4x64avx2
    end interface

    interface
        function vsmc_mkl_brng_threefry2x32sse2() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry2x32sse2
        end function vsmc_mkl_brng_threefry2x32sse2
    end interface

    interface
        function vsmc_mkl_brng_threefry4x32sse2() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry4x32sse2
        end function vsmc_mkl_brng_threefry4x32sse2
    end interface

    interface
        function vsmc_mkl_brng_threefry2x64sse2() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry2x64sse2
        end function vsmc_mkl_brng_threefry2x64sse2
    end interface

    interface
        function vsmc_mkl_brng_threefry4x64sse2() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_threefry4x64sse2
        end function vsmc_mkl_brng_threefry4x64sse2
    end interface

    interface
        function vsmc_mkl_brng_aes128_1x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes128_1x32
        end function vsmc_mkl_brng_aes128_1x32
    end interface

    interface
        function vsmc_mkl_brng_aes128_2x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes128_2x32
        end function vsmc_mkl_brng_aes128_2x32
    end interface

    interface
        function vsmc_mkl_brng_aes128_4x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes128_4x32
        end function vsmc_mkl_brng_aes128_4x32
    end interface

    interface
        function vsmc_mkl_brng_aes128_8x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes128_8x32
        end function vsmc_mkl_brng_aes128_8x32
    end interface

    interface
        function vsmc_mkl_brng_aes128_1x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes128_1x64
        end function vsmc_mkl_brng_aes128_1x64
    end interface

    interface
        function vsmc_mkl_brng_aes128_2x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes128_2x64
        end function vsmc_mkl_brng_aes128_2x64
    end interface

    interface
        function vsmc_mkl_brng_aes128_4x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes128_4x64
        end function vsmc_mkl_brng_aes128_4x64
    end interface

    interface
        function vsmc_mkl_brng_aes128_8x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes128_8x64
        end function vsmc_mkl_brng_aes128_8x64
    end interface

    interface
        function vsmc_mkl_brng_aes192_1x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes192_1x32
        end function vsmc_mkl_brng_aes192_1x32
    end interface

    interface
        function vsmc_mkl_brng_aes192_2x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes192_2x32
        end function vsmc_mkl_brng_aes192_2x32
    end interface

    interface
        function vsmc_mkl_brng_aes192_4x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes192_4x32
        end function vsmc_mkl_brng_aes192_4x32
    end interface

    interface
        function vsmc_mkl_brng_aes192_8x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes192_8x32
        end function vsmc_mkl_brng_aes192_8x32
    end interface

    interface
        function vsmc_mkl_brng_aes192_1x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes192_1x64
        end function vsmc_mkl_brng_aes192_1x64
    end interface

    interface
        function vsmc_mkl_brng_aes192_2x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes192_2x64
        end function vsmc_mkl_brng_aes192_2x64
    end interface

    interface
        function vsmc_mkl_brng_aes192_4x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes192_4x64
        end function vsmc_mkl_brng_aes192_4x64
    end interface

    interface
        function vsmc_mkl_brng_aes192_8x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes192_8x64
        end function vsmc_mkl_brng_aes192_8x64
    end interface

    interface
        function vsmc_mkl_brng_aes256_1x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes256_1x32
        end function vsmc_mkl_brng_aes256_1x32
    end interface

    interface
        function vsmc_mkl_brng_aes256_2x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes256_2x32
        end function vsmc_mkl_brng_aes256_2x32
    end interface

    interface
        function vsmc_mkl_brng_aes256_4x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes256_4x32
        end function vsmc_mkl_brng_aes256_4x32
    end interface

    interface
        function vsmc_mkl_brng_aes256_8x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes256_8x32
        end function vsmc_mkl_brng_aes256_8x32
    end interface

    interface
        function vsmc_mkl_brng_aes256_1x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes256_1x64
        end function vsmc_mkl_brng_aes256_1x64
    end interface

    interface
        function vsmc_mkl_brng_aes256_2x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes256_2x64
        end function vsmc_mkl_brng_aes256_2x64
    end interface

    interface
        function vsmc_mkl_brng_aes256_4x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes256_4x64
        end function vsmc_mkl_brng_aes256_4x64
    end interface

    interface
        function vsmc_mkl_brng_aes256_8x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_aes256_8x64
        end function vsmc_mkl_brng_aes256_8x64
    end interface

    interface
        function vsmc_mkl_brng_ars_1x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ars_1x32
        end function vsmc_mkl_brng_ars_1x32
    end interface

    interface
        function vsmc_mkl_brng_ars_2x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ars_2x32
        end function vsmc_mkl_brng_ars_2x32
    end interface

    interface
        function vsmc_mkl_brng_ars_4x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ars_4x32
        end function vsmc_mkl_brng_ars_4x32
    end interface

    interface
        function vsmc_mkl_brng_ars_8x32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ars_8x32
        end function vsmc_mkl_brng_ars_8x32
    end interface

    interface
        function vsmc_mkl_brng_ars_1x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ars_1x64
        end function vsmc_mkl_brng_ars_1x64
    end interface

    interface
        function vsmc_mkl_brng_ars_2x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ars_2x64
        end function vsmc_mkl_brng_ars_2x64
    end interface

    interface
        function vsmc_mkl_brng_ars_4x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ars_4x64
        end function vsmc_mkl_brng_ars_4x64
    end interface

    interface
        function vsmc_mkl_brng_ars_8x64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_ars_8x64
        end function vsmc_mkl_brng_ars_8x64
    end interface

    interface
        function vsmc_mkl_brng_rdrand16() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_rdrand16
        end function vsmc_mkl_brng_rdrand16
    end interface

    interface
        function vsmc_mkl_brng_rdrand32() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_rdrand32
        end function vsmc_mkl_brng_rdrand32
    end interface

    interface
        function vsmc_mkl_brng_rdrand64() bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int) :: vsmc_mkl_brng_rdrand64
        end function vsmc_mkl_brng_rdrand64
    end interface

    enum, bind(c)
        enumerator VSMC_RESAMPLE_SCHEME_MULTINOMIAL
        enumerator VSMC_RESAMPLE_SCHEME_STRATIFIED
        enumerator VSMC_RESAMPLE_SCHEME_SYSTEMATIC
        enumerator VSMC_RESAMPLE_SCHEME_RESIDUAL
        enumerator VSMC_RESAMPLE_SCHEME_RESIDUAL_STRATIFIED
        enumerator VSMC_RESAMPLE_SCHEME_RESIDUAL_SYSTEMATIC
    end enum

    interface
        subroutine vsmc_resample_trans_u01_replication(&
                m, n, weight, u01, replication) bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: weight
            real(kind = c_double), dimension(*) :: u01
            integer(kind = c_int), dimension(*) :: replication
        end subroutine vsmc_resample_trans_u01_replication
    end interface

    interface
        subroutine vsmc_resample_trans_u01_index(&
                m, n, weight, u01, src_idx) bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: weight
            real(kind = c_double), dimension(*) :: u01
            integer(kind = c_int), dimension(*) :: src_idx
        end subroutine vsmc_resample_trans_u01_index
    end interface

    interface
        subroutine vsmc_resample_trans_replication_index(&
                m, n, replication, src_idx) bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: replication
            integer(kind = c_int), dimension(*) :: src_idx
        end subroutine vsmc_resample_trans_replication_index
    end interface

    interface
        subroutine vsmc_resample_trans_index_replication(&
                m, n, src_idx, replication) bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            integer(kind = c_int), dimension(*) :: src_idx
            integer(kind = c_int), dimension(*) :: replication
        end subroutine vsmc_resample_trans_index_replication
    end interface

    interface
        function vsmc_resample_trans_residual(&
                m, n, weight, resid, integ) bind(c)
            use, intrinsic :: iso_c_binding
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            real(kind = c_double), dimension(*) :: weight
            real(kind = c_double), dimension(*) :: resid
            integer(kind = c_int), dimension(*) :: integ
        end function vsmc_resample_trans_residual
    end interface

    interface
        subroutine vsmc_resample_multinomial(&
                m, n, rng, weight, replication) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            type(vsmc_rng) :: rng
            real(kind = c_double), dimension(*) :: weight
            integer(kind = c_int), dimension(*) :: replication
        end subroutine vsmc_resample_multinomial
    end interface

    interface
        subroutine vsmc_resample_stratified(&
                m, n, rng, weight, replication) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            type(vsmc_rng) :: rng
            real(kind = c_double), dimension(*) :: weight
            integer(kind = c_int), dimension(*) :: replication
        end subroutine vsmc_resample_stratified
    end interface

    interface
        subroutine vsmc_resample_systematic(&
                m, n, rng, weight, replication) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            type(vsmc_rng) :: rng
            real(kind = c_double), dimension(*) :: weight
            integer(kind = c_int), dimension(*) :: replication
        end subroutine vsmc_resample_systematic
    end interface

    interface
        subroutine vsmc_resample_residual(&
                m, n, rng, weight, replication) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            type(vsmc_rng) :: rng
            real(kind = c_double), dimension(*) :: weight
            integer(kind = c_int), dimension(*) :: replication
        end subroutine vsmc_resample_residual
    end interface

    interface
        subroutine vsmc_resample_residual_stratified(&
                m, n, rng, weight, replication) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            type(vsmc_rng) :: rng
            real(kind = c_double), dimension(*) :: weight
            integer(kind = c_int), dimension(*) :: replication
        end subroutine vsmc_resample_residual_stratified
    end interface

    interface
        subroutine vsmc_resample_residual_systematic(&
                m, n, rng, weight, replication) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            type(vsmc_rng) :: rng
            real(kind = c_double), dimension(*) :: weight
            integer(kind = c_int), dimension(*) :: replication
        end subroutine vsmc_resample_residual_systematic
    end interface

    interface
        subroutine vsmc_resample(&
                m, n, rng, weight, replication, scheme) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            integer(kind = c_int), value :: m
            integer(kind = c_int), value :: n
            type(vsmc_rng) :: rng
            real(kind = c_double), dimension(*) :: weight
            integer(kind = c_int), dimension(*) :: replication
            integer(kind = kind(VSMC_RESAMPLE_SCHEME_MULTINOMIAL)) scheme
        end subroutine vsmc_resample
    end interface

end module vsmc
