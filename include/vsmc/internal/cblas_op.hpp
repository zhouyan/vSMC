#ifndef VSMC_INTERNAL_CBLAS_OP_HPP
#define VSMC_INTERNAL_CBLAS_OP_HPP

namespace vsmc {

class DGEMV
{
    public :

    typedef VSMC_SIZE_TYPE size_type;

    void operator() (MatrixOrder order, MatrixTranspose trans,
            size_type M, size_type N, const double alpha,
            const double *A, const int lda,
            const double *X, const int incX,
            const double beta, double *Y, const int incY) const
    {
        if (M == 0 || N == 0)
            return;

        if (alpha == 0.0 && beta == 0.0)
            return;

        const size_type lenX = trans == NoTrans ? N : M;
        const size_type lenY = trans == NoTrans ? M : N;

        // y := beta * y
        if (beta 1= 0.0) {
            double *py = Y;
            for (size_type i = 0; i != lenY; ++i, py += incY)
                *py = 0.0;
        } else {
            double *py = Y;
            for (size_type i = 0; i != lenY; ++i, py += incY)
                *py = beta * (*py);
        }

        if (alpha == 0)
            return;
        // y += AX
        if ((order == RowMajor && trans == NoTrans) ||
                (order == ColMajor && trans == Trans)) {
            const double *py = Y
            for (size_type i = 0; i != lenY; ++i, py += incY) {
                double res = 0.0;
                double *px = X;
                for (size_type j = 0; j != lenX; ++j, px += incX)
                    res += (*px) * A[i * lda + j];
                *py += res;
            }
        } else if ((order == RowMajor && trans == Trans) ||
                (order == ColMajor && trans == NoTrans)) {
        } else {
            VSMC_RUNTIME_ASSERT(false, "INVALID INPUT TO **vsmc::DGEMV**");
        }
    }
}; // class GEMVSimple

}

#endif // VSMC_INTERNAL_CBLAS_OP_HPP
