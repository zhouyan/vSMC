#ifndef VSMC_MPI_BACKEND_MPI_HPP
#define VSMC_MPI_BACKEND_MPI_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/weight.hpp>

#include <boost/mpi/collectives.hpp>
#include <boost/mpi/communicator.hpp>
#include <boost/mpi/environment.hpp>

namespace vsmc {

/// \brief Particle::weight_set_type subtype using MPI
/// \ingroup MPI
template <typename BaseState>
class WeightSetMPI : public WeightSet<BaseState>
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;

    explicit WeightSetMPI (size_type N) :
        WeightSet<BaseState>(N), ess_(static_cast<double>(N) * world_.size())
    {}

    size_type resample_size () const {return this->size() * world_.size();}

    template <typename OutputIter>
    OutputIter read_resample_weight (OutputIter first) const
    {
        world_.barrier();

        boost::mpi::all_gather(world_, this->weight_vec(), weight_gather_);
        for (int r = 0; r != world_.size(); ++r)
            for (size_type i = 0; i != this->size(); ++i, ++first)
                *first = weight_gather_[r][i];

        world_.barrier();

        return first;
    }

    template <typename RandomIter>
    RandomIter read_resample_weight (RandomIter first, int stride) const
    {
        world_.barrier();

        boost::mpi::all_gather(world_, this->weight_vec(), weight_gather_);
        for (int r = 0; r != world_.size(); ++r)
            for (size_type i = 0; i != this->size(); ++i, first += stride)
                *first = weight_gather_[r][i];

        world_.barrier();

        return first;
    }

    void set_equal_weight ()
    {
        world_.barrier();

        ess_ = static_cast<double>(this->size()) * world_.size();
        double ew = 1 / ess_;
        std::vector<double> &weight = this->weight_vec();
        std::vector<double> &log_weight = this->log_weight_vec();
        for (size_type i = 0; i != this->size(); ++i) {
            weight[i] = ew;
            log_weight[i] = 0;
        }

        world_.barrier();
    }

    double ess () const {return ess_;}

    private :

    boost::mpi::communicator world_;
    double ess_;
    mutable std::vector<std::vector<double> > weight_gather_;

    void normalize_weight ()
    {
        world_.barrier();

        std::vector<double> &weight = this->weight_vec();

        double lcoeff = 0;
        for (size_type i = 0; i != this->size(); ++i)
            lcoeff += weight[i];
        double gcoeff = 0;
        boost::mpi::all_reduce(world_, lcoeff, gcoeff, std::plus<double>());
        gcoeff = 1 / gcoeff;
        for (size_type i = 0; i != this->size(); ++i)
            weight[i] *= gcoeff;

        double less = 0;
        for (size_type i = 0; i != this->size(); ++i)
            less += weight[i] * weight[i];
        double gess = 0;
        boost::mpi::all_reduce(world_, less, gess, std::plus<double>());
        gess = 1 / gess;
        ess_ = gess;

        world_.barrier();
    }
}; // class WeightSetMPI

/// \brief Particle::value_type subtype using MPI
/// \ingroup MPI
///
/// \details
/// The tag `boost::mpi::environment::max_tag()` is reserved by vSMC for copy
/// particles
template <typename BaseState>
class StateMPI : public BaseState
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;
    typedef WeightSetMPI<BaseState> weight_set_type;

    explicit StateMPI (size_type N) :
        BaseState(N), offset_(N * static_cast<size_type>(world_.rank())),
        copy_particle_tag_(boost::mpi::environment::max_tag()) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH_MPI;

        world_.barrier();

        copy_from_.resize(N);
        if (world_.rank() == 0) {
            for (size_type i = 0; i != N; ++i)
                copy_from_[i] = copy_from[i];
        }
        boost::mpi::broadcast(world_, copy_from_, 0);

        for (size_type to = 0; to != N; ++to) {
            size_type from = copy_from_[to];
            if (is_local(to) && is_local(from)) {
                size_type lto = local_id(to);
                size_type lfrom = local_id(from);
                this->copy_particle(lfrom, lto);
            } else if (is_local(to)) {
                size_type lto = local_id(to);
                typename BaseState::state_pack_type pack(
                        this->state_pack(lto));
                world_.recv(rank(from), copy_particle_tag_, pack);
                this->state_unpack(lto, pack);
            } else if (is_local(from)) {
                size_type lfrom = local_id(from);
                typename BaseState::state_pack_type pack(
                        this->state_pack(lfrom));
                world_.send(rank(to), copy_particle_tag_, pack);
            }
        }

        world_.barrier();
    }

    boost::mpi::communicator &world () {return world_;}

    const boost::mpi::communicator &world () const {return world_;}

    protected :

    size_type offset () const {return offset_;}

    bool is_local (size_type global_id) const
    {return global_id >= offset_ && global_id < this->size() + offset_;}

    size_type local_id (size_type global_id) const
    {return global_id - offset_;}

    size_type global_id (size_type local_id) const
    {return local_id + offset_;}

    int rank (size_type global_id) const
    {return static_cast<int>(global_id / this->size());}

    private :

    boost::mpi::communicator world_;
    size_type offset_;
    int copy_particle_tag_;
    std::vector<size_type> copy_from_;
}; // class StateMPI

} // namespace vsmc

#endif // VSMC_MPI_BACKEND_MPI_HPP
