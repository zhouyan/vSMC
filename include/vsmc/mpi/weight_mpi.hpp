#ifndef VSMC_MPI_WEIGHT_MPI_HPP
#define VSMC_MPI_WEIGHT_MPI_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/weight.hpp>
#include <vsmc/mpi/mpi_manager.hpp>

namespace vsmc {

/// \brief Particle::weight_set_type subtype using MPI
/// \ingroup MPI
template <typename ID>
class WeightSetMPI : public WeightSet
{
    public :

    typedef WeightSet::size_type size_type;

    explicit WeightSetMPI (size_type N) :
        WeightSet(N), world_(MPICommunicator<ID>::instance().get(),
                boost::mpi::comm_duplicate), internal_barrier_(true),
        resample_size_(0)
    {
        boost::mpi::all_reduce(
                world_, N, resample_size_, std::plus<size_type>());
        this->set_ess(static_cast<double>(resample_size_));
        barrier();
    }

    size_type resample_size () const {return resample_size_;}

    template <typename OutputIter>
    OutputIter read_resample_weight (OutputIter first) const
    {
        barrier();
        gather_resample_weight();
        if (world_.rank() == 0) {
            for (int r = 0; r != world_.size(); ++r) {
                const size_type N = weight_all_[r].size();
                const double *const wptr = &weight_all_[r][0];
                for (size_type i = 0; i != N; ++i, ++first)
                    *first = wptr[i];
            }
        }
        barrier();

        return first;
    }

    template <typename RandomIter>
    RandomIter read_resample_weight (RandomIter first, int stride) const
    {
        barrier();
        gather_resample_weight();
        if (world_.rank() == 0) {
            for (int r = 0; r != world_.size(); ++r) {
                const size_type N = weight_all_[r].size();
                const double *const wptr = &weight_all_[r][0];
                for (size_type i = 0; i != N; ++i, first += stride)
                    *first = wptr[i];
            }
        }
        barrier();

        return first;
    }

    double *read_resample_weight (double *first) const
    {
        barrier();
        gather_resample_weight();
        if (world_.rank() == 0) {
            for (int r = 0; r != world_.size(); ++r) {
                const size_type N = weight_all_[r].size();
                const double *const wptr = &weight_all_[r][0];
                VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_OUT(
                        first - wptr, N, WeightSetMPI::read_resample_weight);
                std::memcpy(first, wptr, sizeof(double) * N);
                first += N;
            }
        }
        barrier();

        return first;
    }

    void set_equal_weight ()
    {
        barrier();

        const size_type N = static_cast<size_type>(this->size());
        double *const weight = this->weight_ptr();
        double *const log_weight = this->log_weight_ptr();

        this->set_ess(static_cast<double>(resample_size_));
        double ew = 1 / this->ess();
        for (size_type i = 0; i != N; ++i) {
            weight[i] = ew;
            log_weight[i] = 0;
        }

        barrier();
    }

    /// \brief A duplicated MPI communicator for this weight set object
    const boost::mpi::communicator &world () const {return world_;}

    void barrier () const {if (internal_barrier_) world_.barrier();}

    void internal_barrier (bool use) {internal_barrier_ = use;}

    protected :

    void normalize_log_weight ()
    {
        barrier();

        const size_type N = static_cast<size_type>(this->size());
        double *const log_weight = this->log_weight_ptr();

        double lmax_weight = log_weight[0];
        for (size_type i = 0; i != N; ++i)
            if (log_weight[i] > lmax_weight)
                lmax_weight = log_weight[i];
        double gmax_weight = 0;
        boost::mpi::all_reduce(world_, lmax_weight, gmax_weight,
                boost::mpi::maximum<double>());
        for (size_type i = 0; i != N; ++i)
            log_weight[i] -= gmax_weight;

        barrier();
    }

    void normalize_weight ()
    {
        barrier();

        const size_type N = static_cast<size_type>(this->size());
        double *const weight = this->weight_ptr();

        double lcoeff = 0;
        for (size_type i = 0; i != N; ++i)
            lcoeff += weight[i];
        double gcoeff = 0;
        boost::mpi::all_reduce(world_, lcoeff, gcoeff, std::plus<double>());
        gcoeff = 1 / gcoeff;
        for (size_type i = 0; i != N; ++i)
            weight[i] *= gcoeff;

        double less = 0;
        for (size_type i = 0; i != N; ++i)
            less += weight[i] * weight[i];
        double gess = 0;
        boost::mpi::all_reduce(world_, less, gess, std::plus<double>());
        gess = 1 / gess;
        this->set_ess(gess);

        barrier();
    }

    double compute_ess (const double *first, bool use_log) const
    {
        using std::exp;

        barrier();

        const size_type N = static_cast<size_type>(this->size());
        const double *const weight = this->weight_ptr();

        buffer_.resize(N);
        double *const bptr = &buffer_[0];
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_IN(
                first - bptr, N, WeightSetMPI::ess);
        std::memcpy(bptr, first, sizeof(double) * N);

        if (use_log) {
            for (size_type i = 0; i != N; ++i)
                bptr[i] = exp(bptr[i]);
        }

        double lcoeff = 0;
        for (size_type i = 0; i != N; ++i) {
            bptr[i] *= weight[i];
            lcoeff += bptr[i];
        }
        double gcoeff = 0;
        boost::mpi::all_reduce(world_, lcoeff, gcoeff, std::plus<double>());
        gcoeff = 1 / gcoeff;
        for (size_type i = 0; i != N; ++i)
            bptr[i] *= gcoeff;

        double less = 0;
        for (size_type i = 0; i != N; ++i)
            less += bptr[i] * bptr[i];
        double gess = 0;
        boost::mpi::all_reduce(world_, less, gess, std::plus<double>());
        gess = 1 / gess;

        barrier();

        return gess;
    }

    double compute_cess (const double *first, bool use_log) const
    {
        using std::exp;

        barrier();

        const size_type N = static_cast<size_type>(this->size());
        const double *const weight = this->weight_ptr();

        buffer_.resize(N);
        double *const bptr = &buffer_[0];
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_IN(
                first - bptr, N, WeightSetMPI::ess);
        std::memcpy(bptr, first, sizeof(double) * N);

        if (use_log) {
            for (size_type i = 0; i != N; ++i)
                bptr[i] = exp(bptr[i]);
        }

        double labove = 0;
        double lbelow = 0;
        for (size_type i = 0; i != N; ++i) {
            labove += weight[i] * bptr[i];
            lbelow += weight[i] * bptr[i] * bptr[i];
        }
        double gabove = 0;
        double gbelow = 0;
        boost::mpi::all_reduce(world_, labove, gabove, std::plus<double>());
        boost::mpi::all_reduce(world_, lbelow, gbelow, std::plus<double>());

        barrier();

        return gabove * gabove / gbelow;
    }

    private :

    boost::mpi::communicator world_;
    bool internal_barrier_;
    size_type resample_size_;
    mutable std::vector<double> buffer_;
    mutable std::vector<std::vector<double> > weight_all_;

    void gather_resample_weight () const
    {
        if (world_.rank() == 0)
            boost::mpi::gather(world_, this->weight_vec(), weight_all_, 0);
        else
            boost::mpi::gather(world_, this->weight_vec(), 0);
    }
}; // class WeightSetMPI

} // namespace vsmc

#endif // VSMC_MPI_WEIGHT_MPI_HPP
