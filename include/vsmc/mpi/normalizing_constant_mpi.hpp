#ifndef VSMC_MPI_NORMALIZING_CONSTANT_MPI_HPP
#define VSMC_MPI_NORMALIZING_CONSTANT_MPI_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/normalizing_constant.hpp>
#include <vsmc/mpi/manager.hpp>

namespace vsmc {

/// \brief Calculating normalizing constant ratio using MPI
/// \ingroup MPI
template <typename ID>
class NormalizingConstantMPI : public NormalizingConstant
{
    public :

    NormalizingConstantMPI (std::size_t N) :
        NormalizingConstant(N), world_(MPICommunicator<ID>::instance().get(),
                boost::mpi::comm_duplicate), internal_barrier_(true) {}

    const boost::mpi::communicator &world () const {return world_;}

    void barrier () const {if (internal_barrier_) world_.barrier();}

    void internal_barrier (bool use) {internal_barrier_ = use;}

    protected:

    double inc_zconst (std::size_t N,
            const double *weight, const double *inc_weight) const
    {
        double linc = NormalizingConstant::inc_zconst(N, weight, inc_weight);
        double ginc = 0;
        boost::mpi::all_reduce(world_, linc, ginc, std::plus<double>());
        barrier();

        return ginc;
    }

    private :

    boost::mpi::communicator world_;
    bool internal_barrier_;
}; // class NormalizingConstantMPI

} // namespace vsmc

#endif // VSMC_MPI_NORMALIZING_CONSTANT_MPI_HPP
