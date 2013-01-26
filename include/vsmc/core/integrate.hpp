#ifndef VSMC_CORE_INTEGRATE_HPP
#define VSMC_CORE_INTEGRATE_HPP

#include <vsmc/internal/common.hpp>

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

namespace vsmc {

namespace internal {

/// \brief Check if a pointer is aligned to 16 bytes
/// \ingroup Core
inline bool is_sse_aligned (void *ptr)
{
    return ((unsigned long) ptr & 15) == 0;
}

} // namespace vsmc::internal

/// \brief Compute the importance sampling integration of univariate variable
/// \ingroup Core
class Integrate1
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
}; // Integrate1

/// \brief Compute the importance sampling integration of multivariate variable
/// \ingroup Core
class IntegrateD
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
}; // class IntegrateD

template <typename Derived>
class IntegrateNumeric
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef cxx11::function<double (double)> eval_type;

    double operator() (size_type N, const double *grid) const
    {
        double integral = 0;
        for (size_type i = 1; i != N; ++i)
            integral += static_cast<const Derived *>(this)->
                integrate_segment(grid[i - 1], grid[i]);

        return integral;
    }

    template <typename InputIter>
    double operator() (size_type N, InputIter grid) const
    {
        grid_.resize(N);
        for (size_type i = 0; i != N; ++i, ++grid)
            grid_[i] = *grid;

        return integrate(N, &grid_[0]);
    }

    template <typename InputIter>
    double operator() (InputIter grid_begin, InputIter grid_end) const
    {
        grid_.clear();
        for (InputIter iter = grid_begin; iter != grid_end; ++iter)
            grid_.push_back(*iter);

        return integrate(static_cast<size_type>(grid_.size()), &grid_[0]);
    }

    private :

    mutable std::vector<double> grid_;
}; // class IntegrateNumeric

template <unsigned Degree>
class IntegrateNewtonCotes :
    public IntegrateNumeric<IntegrateNewtonCotes<Degree> >
{
    public :

    typedef IntegrateNumeric<IntegrateNewtonCotes<Degree> > base_type;
    typedef typename base_type::size_type size_type;
    typedef typename base_type::eval_type eval_type;

    IntegrateNewtonCotes (const eval_type &eval) : eval_(eval)
    {
        VSMC_STATIC_ASSERT_INTEGRATE_NEWTON_COTES_DEGREE(Degree);
    }

    double integrate_segment (double a, double b) const
    {
        return integrate_segment_dispatch(a, b,
                cxx11::integral_constant<unsigned, Degree>());
    }

    private :

    eval_type eval_;

    double integrate_segment_dispatch (double a, double b,
            cxx11::integral_constant<unsigned, 1>) const
    {
        return 0.5 * (b - a) * (eval_(a) + eval_(b));
    }

    double integrate_segment_dispatch (double a, double b,
            cxx11::integral_constant<unsigned, 2>) const
    {
        static const double coeff = 1.0 / 6.0;

        return coeff * (b - a) * (
                eval_(a) + 4 * eval_(a + 0.5 * (b - a)) + eval_(b));
    }

    double integrate_segment_dispatch (double a, double b,
            cxx11::integral_constant<unsigned, 3>) const
    {
        double h = (b - a ) / 3;
        double x1 = a + h;
        double x2 = b - h;

        return 0.125 * (b - a) * (
                eval_(a) + 3 * eval_(x1) + 3 * eval_(x2) + eval_(b));
    }

    double integrate_segment_dispatch (double a, double b,
            cxx11::integral_constant<unsigned, 4>) const
    {
        static const double coeff = 1.0 / 90.0;
        double h = 0.25 * (b - a);
        double x1 = a + h;
        double x2 = a + h * 2;
        double x3 = a + h * 3;

        return coeff * (b - a) * (
                7 * eval_(a) + 32 * eval_(x1) + 12 * eval_(x2) +
                32 * eval_(x3) + 7 * eval_(b));
    }
}; // class IntegrateNewtonCotes

typedef IntegrateNewtonCotes<1> IntegrateTrapezoid;
typedef IntegrateNewtonCotes<2> IntegrateSimpson;
typedef IntegrateNewtonCotes<3> IntegrateSimpson3_8;
typedef IntegrateNewtonCotes<4> IntegrateBoole;

} // namespace vsmc

#endif // VSMC_CORE_INTEGRATE_HPP
