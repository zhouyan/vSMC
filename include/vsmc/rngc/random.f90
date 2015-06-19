! ============================================================================
!  vSMC/include/vsmc/rngc/random.f90
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
!  SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
!  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
!  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
!  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
!  POSSIBILITY OF SUCH DAMAGE.
! ============================================================================

module vsmc_random
    use, intrinsic :: iso_c_binding
    type, bind(c) :: vsmc_rng
        integer(kind = c_int64_t), dimension(128) :: state
    end type vsmc_rng

    interface
        subroutine vsmc_rng_init(rng, seed) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int32_t), value :: seed
        end subroutine vsmc_rng_init
    end interface

    interface
        subroutine vsmc_rng_seed(rng, seed) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int32_t), value :: seed
        end subroutine vsmc_rng_seed
    end interface

    interface
        subroutine vsmc_rng_key(rng, key) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int32_t), dimension(4) :: key
        end subroutine vsmc_rng_key
    end interface

    interface
        subroutine vsmc_rng_ctr(rng, ctr) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int32_t), dimension(4) :: ctr
        end subroutine vsmc_rng_ctr
    end interface

    interface
        subroutine vsmc_rng_gen(rng, n, r) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer(kind = c_int32_t), value :: n
            integer(kind = c_int), dimension(*) :: r
        end subroutine vsmc_rng_gen
    end interface

    interface
        subroutine vsmc_rng_uniform_int(rng, n, r, a, b) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
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
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: a
            real(kind = c_double), value :: b
        end subroutine vsmc_rng_uniform_real
    end interface

    interface
        subroutine vsmc_rng_bernoulli(rng, n, r, p) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
            integer(kind = c_int), dimension(*) :: r
            real(kind = c_double), value :: p
        end subroutine vsmc_rng_bernoulli
    end interface

    interface
        subroutine vsmc_rng_binomial(rng, n, r, t, p) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
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
            integer, value :: n
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
            integer, value :: n
            integer(kind = c_int), dimension(*) :: r
            real(kind = c_double), value :: p
        end subroutine vsmc_rng_geometric
    end interface

    interface
        subroutine vsmc_rng_poisson(rng, n, r, mean) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
            integer(kind = c_int), dimension(*) :: r
            real(kind = c_double), value :: mean
        end subroutine vsmc_rng_poisson
    end interface

    interface
        subroutine vsmc_rng_exponential(rng, n, r, rate) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: rate
        end subroutine vsmc_rng_exponential
    end interface

    interface
        subroutine vsmc_rng_gamma(rng, n, r, shap, scal) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
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
            integer, value :: n
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
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: location
            real(kind = c_double), value :: scal
        end subroutine vsmc_rng_extreme_value
    end interface

    interface
        subroutine vsmc_rng_normal(rng, n, r, mean, sd) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: mean
            real(kind = c_double), value :: sd
        end subroutine vsmc_rng_normal
    end interface

    interface
        subroutine vsmc_rng_lognormal(rng, n, r, logmean, logsd) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: logmean
            real(kind = c_double), value :: logsd
        end subroutine vsmc_rng_lognormal
    end interface

    interface
        subroutine vsmc_rng_chi_squared(rng, n, r, df) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: df
        end subroutine vsmc_rng_chi_squared
    end interface

    interface
        subroutine vsmc_rng_cauchy(rng, n, r, location, scal) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: location
            real(kind = c_double), value :: scal
        end subroutine vsmc_rng_cauchy
    end interface

    interface
        subroutine vsmc_rng_fisher_f(rng, n, r, df1, df2) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
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
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: df
        end subroutine vsmc_rng_student_t
    end interface

    interface
        subroutine vsmc_rng_stable(rng, n, r, &
                stability, skewness, location, scal) bind(c)
            use, intrinsic :: iso_c_binding
            import :: vsmc_rng
            type(vsmc_rng) :: rng
            integer, value :: n
            real(kind = c_double), dimension(*) :: r
            real(kind = c_double), value :: stability
            real(kind = c_double), value :: skewness
            real(kind = c_double), value :: location
            real(kind = c_double), value :: scal
        end subroutine vsmc_rng_stable
    end interface
end module vsmc_random
