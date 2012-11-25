#ifndef VSMC_UTILITY_CBLAS_OP_HPP
#define VSMC_UTILITY_CBLAS_OP_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

template <typename T>
class DDOT
{
    public :

    typedef typename traits::SizeTypeTrait<T>::type size_type;

    double operator() (size_type N,
            const double *X, const int incX, const double *Y, const int incY)
    {
        if (N == 0)
            return 0;

        VSMC_RUNTIME_ASSERT((incX > 0),
                "NON-POSITIVE STRIDE OF X IN **vsmc::DDOT**");
        VSMC_RUNTIME_ASSERT((incY > 0),
                "NON-POSITIVE STRIDE OF Y IN **vsmc::DDOT**");

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
}; // class DDOT

template <typename T>
class DGEMV
{
    public :

    typedef typename traits::SizeTypeTrait<T>::type size_type;

    void operator() (MatrixOrder order, MatrixTranspose trans,
            size_type M, size_type N, const double alpha,
            const double *A, const int lda,
            const double *X, const int incX,
            const double beta, double *Y, const int incY) const
    {
        if (M == 0 || N == 0)
            return;

        if (alpha == 0 && beta == 0)
            return;

        VSMC_RUNTIME_ASSERT((lda > 0),
                "NON-POSITIVE STRIDE OF A IN **vsmc::DGEMV**");
        VSMC_RUNTIME_ASSERT((incX > 0),
                "NON-POSITIVE STRIDE OF X IN **vsmc::DGEMV**");
        VSMC_RUNTIME_ASSERT((incY > 0),
                "NON-POSITIVE STRIDE OF Y IN **vsmc::DGEMV**");

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
            VSMC_RUNTIME_ASSERT(false, "INVALID INPUT TO **vsmc::DGEMV**");
        }
    }
}; // class DGEMV

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(DGEMVType, dgemv_type, DGEMV<T>);
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(DDOTType, ddot_type, DDOT<T>);

#endif // VSMC_UTILITY_CBLAS_OP_HPP
