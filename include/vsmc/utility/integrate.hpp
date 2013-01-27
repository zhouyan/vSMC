#ifndef VSMC_CORE_INTEGRATE_HPP
#define VSMC_CORE_INTEGRATE_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>

#include <vsmc/cxx11/functional.hpp>
#include <vsmc/cxx11/type_traits.hpp>

#include <cstddef>
#include <vector>

#undef VSMC_USE_CBLAS
#undef VSMC_INTEGRATE_INT

#if VSMC_USE_MKL // MKL
#include <mkl_cblas.h>
#define VSMC_USE_CBLAS 1
#define VSMC_INTEGRATE_INT MKL_INT
#elif VSMC_USE_VECLIB // vecLib
#include <vecLib/cblas.h>
#define VSMC_USE_CBLAS 1
#define VSMC_INTEGRATE_INT int
#elif VSMC_USE_GENERIC_CBLAS // Generic CBlas
#define VSMC_USE_CBLAS 1
#ifdef VSMC_GENERIC_CBLAS_INT
#define VSMC_INTEGRATE_INT VSMC_GENERIC_CBLAS_INT
#else
#define VSMC_INTEGRATE_INT int
#endif
#elif VSMC_USE_ARMADILLO // Armadillo
#include <armadillo>
#define VSMC_INTEGRATE_INT arma::blas_int
#elif VSMC_USE_EIGEN // Eigen
#include <Eigen/Dense>
#define VSMC_INTEGRATE_INT EIGEN_DEFAULT_DENSE_INDEX_TYPE
#else // No known CBlas
#define VSMC_INTEGRATE_INT VSMC_SIZE_TYPE
#endif

namespace vsmc { namespace internal {

inline bool is_sse_aligned (void *ptr)
{
    return ((unsigned long) ptr & 15) == 0;
}

} } // namespace vsmc::internal

namespace vsmc { namespace integrate {

/// \brief Compute the importance sampling integration of univariate variable
/// \ingroup Integrate
class ImportanceSampling1
{
    public :

    typedef VSMC_INTEGRATE_INT size_type;

    /// \brief Compute the importance sampling integration
    ///
    /// \param N Number of particles
    /// \param hX A `N` vector \f$(h(X_i))\f$
    /// \param W Normalized weights
    /// \return The importance sampling estiamte
    double operator() (size_type N, const double *hX, const double *W) const
    {
        if (N == 0)
            return 0;
#if VSMC_USE_CBLAS
        return cblas_ddot(N, hX, 1, W, 1);
#elif VSMC_USE_ARMADILLO
        return arma::dot(arma::vec(hX, N), arma::vec(W, N));
#elif VSMC_USE_EIGEN
        if (internal::is_sse_aligned((void *) hX) &&
                internal::is_sse_aligned((void *) W)) {
            Eigen::Map<const Eigen::VectorXd, Eigen::Aligned> hXEigen(hX, N);
            Eigen::Map<const Eigen::VectorXd, Eigen::Aligned> WEigen(W, N);
            return hXEigen.dot(WEigen);
        } else {
            Eigen::Map<const Eigen::VectorXd> hXEigen(hX, N);
            Eigen::Map<const Eigen::VectorXd> WEigen(W, N);
            return hXEigen.dot(WEigen);
        }
#else
        double res = 0;
        for (size_type i = 0; i != N; ++i)
            res += hX[i] * W[i];
        return res;
#endif
    }
}; // ImportanceSampling1

/// \brief Compute the importance sampling integration of multivariate variable
/// \ingroup Integrate
class ImportanceSamplingD
{
    public :

    typedef VSMC_INTEGRATE_INT size_type;

    /// \brief Compute the importance sampling integration
    ///
    /// \param N Number of particles
    /// \param dim Number of variables
    /// \param hX A `N` by `dim` row major matrix, each row `i` contains
    /// \f$h(X_i) = (h_1(X_i), h_2(X_i), \dots, h_d(X_i))\f$
    /// \param W Normalized weights
    /// \param Eh The importance sampling estiamtes of \f$E[h(X)]\f$
    void operator() (size_type N, size_type dim,
            const double *hX, const double *W, double *Eh) const
    {
        if (N == 0)
            return;
#if VSMC_USE_CBLAS
        cblas_dgemv(CblasColMajor, CblasNoTrans,
                dim, N, 1, hX, dim, W, 1, 0, Eh, 1);
#elif VSMC_USE_ARMADILLO
        arma::vec res(Eh, dim, false);
        res = arma::mat(hX, dim, N) * arma::vec(W, N);
#elif VSMC_USE_EIGEN
        if (internal::is_sse_aligned((void *) hX) &&
                internal::is_sse_aligned((void *) W) &&
                internal::is_sse_aligned((void *) Eh)) {
            Eigen::Map<const Eigen::Matrix<
                double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>,
                Eigen::Aligned> hXEigen(hX, dim, N);
            Eigen::Map<const Eigen::VectorXd, Eigen::Aligned> WEigen(W, N);
            Eigen::Map<Eigen::VectorXd, Eigen::Aligned> res(Eh, dim);
            res = hXEigen * WEigen;
        } else {
            Eigen::Map<const Eigen::Matrix<
                double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> >
                hXEigen(hX, dim, N);
            Eigen::Map<const Eigen::VectorXd> WEigen(W, N);
            Eigen::Map<Eigen::VectorXd> res(Eh, dim);
            res = hXEigen * WEigen;
        }
#else
        for (size_type d = 0; d != dim; ++d)
            Eh[d] = 0;
        for (size_type i = 0; i != N; ++i) {
            double w = W[i];
            for (size_type d = 0; d != dim; ++d)
                Eh[d] += w * hX[i * dim + d];
        }
#endif
    }
}; // class ImportanceSamplingD

/// \brief Base numerical integration
/// \ingroup Integrate
template <typename Derived>
class NumericBase
{
    protected :

    typedef VSMC_SIZE_TYPE size_type;
    typedef cxx11::function<double (double)> eval_type;

    NumericBase () {}
    NumericBase (const NumericBase<Derived> &) {}
    NumericBase<Derived> &operator=
        (const NumericBase<Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~NumericBase () {}

    double integrate_segment (double a, double b, const eval_type &eval)
    {
        return integrate_segment_dispatch(a, b, eval,
                &Derived::integrate_segment);
    }

    private :

    template <typename D>
    double integrate_segment_dispatch (double a, double b,
            const eval_type &eval,
            double (D::*) (double, double, const eval_type &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(NumericBase);
        return static_cast<Derived *>(this)->integrate_segment(a, b, eval);
    }

    double integrate_segment_dispatch (double a, double b,
            const eval_type &eval,
            double (*) (double, double, const eval_type &))
    {
        return Derived::integrate_segment(a, b, eval);
    }

    double integrate_segment_dispatch (double a, double b,
            double (NumericBase::*) (double, double, const eval_type &))
    { VSMC_STATIC_ASSERT_NO_IMPL(integrate_segment); return 0;}
}; // class NumericBase

/// \brief Base numerical integration class with virtual interface
/// \ingroup Integrate
template <>
class NumericBase<VBase>
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef cxx11::function<double (double)> eval_type;

    protected :

    NumericBase () {}
    NumericBase (const NumericBase<VBase> &) {}
    NumericBase<VBase> &operator=
        (const NumericBase<VBase> &) {return *this;}
    virtual ~NumericBase () {}

    virtual double integrate_segment (double, double, const eval_type &) = 0;
}; // class NumericBase<VBase>

/// \brief Compute numerical integration using the Newton-Cotes formulae
/// \ingroup Integrate
template <unsigned Degree, template <typename> class NumericImpl>
class NumericNewtonCotes :
    public NumericImpl<NumericNewtonCotes<Degree, NumericImpl> >
{
    public :

    typedef NumericImpl<NumericNewtonCotes<Degree, NumericImpl> >
        integrate_impl_type;
    typedef typename integrate_impl_type::size_type size_type;
    typedef typename integrate_impl_type::eval_type eval_type;

    double integrate_segment (double a, double b, const eval_type &eval) const
    {
        VSMC_STATIC_ASSERT_NUMERIC_NEWTON_COTES_DEGREE(Degree);
        return integrate_segment_newton_cotes(a, b, eval,
                cxx11::integral_constant<unsigned, Degree>());
    }

    private :

    double integrate_segment_newton_cotes (double a, double b,
            const eval_type &eval,
            cxx11::integral_constant<unsigned, 1>) const
    {
        return 0.5 * (b - a) * (eval(a) + eval(b));
    }

    double integrate_segment_newton_cotes (double a, double b,
            const eval_type &eval,
            cxx11::integral_constant<unsigned, 2>) const
    {
        const double coeff = 1.0 / 6.0;

        return coeff * (b - a) * (
                eval(a) + 4 * eval(a + 0.5 * (b - a)) + eval(b));
    }

    double integrate_segment_newton_cotes (double a, double b,
            const eval_type &eval,
            cxx11::integral_constant<unsigned, 3>) const
    {
        double h = (b - a ) / 3;
        double x1 = a + h;
        double x2 = b - h;

        return 0.125 * (b - a) * (
                eval(a) + 3 * eval(x1) + 3 * eval(x2) + eval(b));
    }

    double integrate_segment_newton_cotes (double a, double b,
            const eval_type &eval,
            cxx11::integral_constant<unsigned, 4>) const
    {
        const double coeff = 1.0 / 90.0;
        double h = 0.25 * (b - a);
        double x1 = a + h;
        double x2 = a + h * 2;
        double x3 = a + h * 3;

        return coeff * (b - a) * (
                7 * eval(a) + 32 * eval(x1) + 12 * eval(x2) +
                32 * eval(x3) + 7 * eval(b));
    }
}; // class NumericNewtonCotes

/// \brief Numerical integration base class
/// \ingroup Integrate
template <typename Derived>
class NumericSEQ : public NumericBase<Derived>
{
    public :

    typedef NumericBase<Derived> integrate_base_type;
    typedef typename integrate_base_type::size_type size_type;
    typedef typename integrate_base_type::eval_type eval_type;

    double operator() (size_type N, const double *grid,
            const eval_type &eval) const
    {
        if (N < 2)
            return 0;

        double integral = 0;
        for (size_type i = 1; i != N; ++i) {
            integral += static_cast<const Derived *>(this)->
                integrate_segment(grid[i - 1], grid[i], eval);
        }

        return integral;
    }
}; // class NumericBase


} } // namespace vsmc::integrate

#endif // VSMC_CORE_INTEGRATE_HPP
