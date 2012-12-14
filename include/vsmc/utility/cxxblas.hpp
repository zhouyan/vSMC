#ifndef VSMC_UTILITY_CXXBLAS_HPP
#define VSMC_UTILITY_CXXBLAS_HPP

#include <vsmc/internal/common.hpp>

#if VSMC_HAS_CBLAS
#include VSMC_CBLAS_HEADER
#endif // VSMC_HAS_CBLAS

namespace vsmc { namespace cxxblas {

template <typename T>
class DDot
{
    public :

#if VSMC_HAS_CBLAS
    typedef VSMC_CBLAS_INT_TYPE size_type;

    double operator() (const size_type N,
            const double *X, const int incX,
            const double *Y, const int incY) const
    {
        return cblas_ddot(N, X, incX, Y, incY);
    }
#else // VSMC_HAS_CBLAS
    typedef typename traits::SizeTypeTrait<T>::type size_type;

    double operator() (const size_type N,
            const double *X, const int incX,
            const double *Y, const int incY) const
    {
        if (N == 0)
            return 0;

        VSMC_RUNTIME_ASSERT((incX > 0),
                "NON-POSITIVE STRIDE OF X IN **vsmc::DDot**");
        VSMC_RUNTIME_ASSERT((incY > 0),
                "NON-POSITIVE STRIDE OF Y IN **vsmc::DDot**");

        if (X == Y && incX == incY) {
            double res = 0;
            if (incX == 1) {
                for (size_type i = 0; i != N; ++i)
                    res += X[i] * X[i];
                return res;
            } else {
                double res = 0;
                for (size_type i = 0; i != N; ++i, X += incX)
                    res += (*X) * (*X);
                return res;
            }
        }

        if (incX == 1 && incY == 1) {
            double res = 0;
            for (size_type i = 0; i != N; ++i, ++X, ++Y)
                res += (*X) * (*Y);
            return res;
        }

        double res = 0;
        for (size_type i = 0; i != N; ++i, X += incX, Y += incY)
            res += (*X) * (*Y);

        return res;
    }
#endif // VSMC_HAS_CBLAS
}; // class DDot

template <typename T>
class DGemv
{
    public :

#if VSMC_HAS_CBLAS
    typedef VSMC_CBLAS_INT_TYPE size_type;

    void operator() (MatrixOrder order, MatrixTranspose trans,
            const size_type M, const size_type N, const double alpha,
            const double *A, const int lda,
            const double *X, const int incX,
            const double beta, double *Y, const int incY) const
    {
        CBLAS_ORDER cblas_order;
        switch (order) {
            case RowMajor :
                cblas_order = CblasRowMajor;
                break;
            case ColMajor :
                cblas_order = CblasColMajor;
        }

        CBLAS_TRANSPOSE cblas_trans;
        switch (trans) {
            case NoTrans :
                cblas_trans = CblasNoTrans;
                break;
            case Trans :
                cblas_trans = CblasTrans;
                break;
            case ConjTrans :
                cblas_trans = CblasConjTrans;
                break;
        }
        cblas_dgemv(cblas_order, cblas_trans, M, N,
                alpha, A, lda, X, incX, beta, Y, incY);
    }
#else // VSMC_HAS_CBLAS
    typedef typename traits::SizeTypeTrait<T>::type size_type;

    void operator() (MatrixOrder order, MatrixTranspose trans,
            const size_type M, const size_type N, const double alpha,
            const double *A, const int lda,
            const double *X, const int incX,
            const double beta, double *Y, const int incY) const
    {
        if (M == 0 || N == 0)
            return;

        if (alpha == 0 && beta == 0)
            return;

        VSMC_RUNTIME_ASSERT((lda > 0),
                "NON-POSITIVE STRIDE OF A IN **vsmc::DGemv**");
        VSMC_RUNTIME_ASSERT((incX > 0),
                "NON-POSITIVE STRIDE OF X IN **vsmc::DGemv**");
        VSMC_RUNTIME_ASSERT((incY > 0),
                "NON-POSITIVE STRIDE OF Y IN **vsmc::DGemv**");

        const size_type lenX = (trans == NoTrans) ? N : M;
        const size_type lenY = (trans == NoTrans) ? M : N;

        // y := beta * y
        if (beta == 0) {
            size_type iy = 0;
            for (size_type i = 0; i != lenY; ++i, iy += incY)
                Y[iy] = 0;
        } else {
            size_type iy = 0;
            for (size_type i = 0; i != lenY; ++i, iy += incY)
                Y[iy] = beta * Y[iy];
        }

        if (alpha == 0)
            return;
        // y += alpha * AX
        if ((order == RowMajor && trans == NoTrans) ||
                (order == ColMajor && trans == Trans)) {
            size_type iy = 0;
            for (size_type i = 0; i != lenY; ++i, iy += incY) {
                double res = 0;
                size_type ix = 0;
                for (size_type j = 0; j != lenX; ++j, ix += incX)
                    res += X[ix] * A[i * lda + j];
                Y[iy] += alpha * res;
            }
        } else if ((order == RowMajor && trans == Trans) ||
                (order == ColMajor && trans == NoTrans)) {
            size_type ix = 0;
            for (size_type j = 0; j != lenX; ++j, ix += incX) {
                double ax = alpha * X[ix];
                size_type iy = 0;
                for (size_type i = 0; i != lenY; ++i, iy += incY)
                    Y[iy] += ax * A[j * lda + i];
            }
        } else {
            VSMC_RUNTIME_ASSERT(false, "INVALID INPUT TO **vsmc::DGemv**");
        }
    }
#endif // VSMC_HAS_CBLAS
}; // class DGemv

} } // namespace vsmc::cxxblas

#endif // VSMC_UTILITY_CXXBLAS_HPP
