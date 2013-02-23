#ifndef VSMC_MPI_BACKEND_MPI_HPP
#define VSMC_MPI_BACKEND_MPI_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/weight.hpp>
#include <boost/mpi.hpp>

namespace vsmc {

/// \brief MPI Communicator
/// \ingroup MPI
///
/// \details
/// Use specialization of the singleton to configure different StateMPI
template <typename ID>
class MPICommunicator
{
    public :

    static MPICommunicator<ID> &instance ()
    {
        static MPICommunicator<ID> comm;

        return comm;
    }

    const MPI_Comm &get () const {return comm_;}

    void set (const MPI_Comm &comm) {comm_ = comm;}

    private :

    MPI_Comm comm_;

    MPICommunicator () : comm_(MPI_COMM_WORLD) {};
    MPICommunicator (const MPICommunicator<ID> &other);
    MPICommunicator<ID> &operator= (const MPICommunicator<ID> &other);
}; // class MPICommunicator

/// \brief Particle::weight_set_type subtype using MPI
/// \ingroup MPI
template <typename BaseState, typename ID>
class WeightSetMPI : public WeightSet<BaseState>
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;

    explicit WeightSetMPI (size_type N) :
        WeightSet<BaseState>(N), world_(MPICommunicator<ID>::instance().get(),
                boost::mpi::comm_duplicate),
        ess_(static_cast<double>(N) * world_.size()) {}

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
template <typename BaseState, typename ID>
class StateMPI : public BaseState
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;
    typedef WeightSetMPI<BaseState, ID> weight_set_type;

    explicit StateMPI (size_type N) :
        BaseState(N), world_(MPICommunicator<ID>::instance().get(),
                boost::mpi::comm_duplicate),
        offset_(N * static_cast<size_type>(world_.rank())),
        copy_tag_(boost::mpi::environment::max_tag()) {}

    /// \brief Copy particles
    ///
    /// \details
    /// The tag `boost::mpi::environment::max_tag()` is reserved by vSMC for
    /// copy particles.
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

        copy_recv_.clear();
        copy_send_.clear();
        int rank_this = world_.rank();
        for (size_type to = 0; to != N; ++to) {
            size_type from = copy_from_[to];
            int rank_recv = rank(to);
            int rank_send = rank(from);
            size_type id_recv = local_id(to);
            size_type id_send = local_id(from);
            if (rank_this == rank_recv && rank_this == rank_send) {
                this->copy_particle(id_send, id_recv);
            } else if (rank_this == rank_recv) {
                copy_recv_.push_back(std::make_pair(rank_send, id_recv));
            } else if (rank_this == rank_send) {
                copy_send_.push_back(std::make_pair(rank_recv, id_send));
            }
        }

        for (int r = 0; r != world_.size(); ++r) {
            if (rank_this == r) {
                for (std::size_t i = 0; i != copy_recv_.size(); ++i) {
                    typename BaseState::state_pack_type pack;
                    world_.recv(copy_recv_[i].first, copy_tag_, pack);
                    this->state_unpack(copy_recv_[i].second, pack);
                }
            } else {
                for (std::size_t i = 0; i != copy_send_.size(); ++i) {
                    if (copy_send_[i].first == r) {
                        world_.send(copy_send_[i].first, copy_tag_,
                                this->state_pack(copy_send_[i].second));
                    }
                }
            }
            world_.barrier();
        }

        world_.barrier();
    }

    /// \brief A duplicated MPI communicator for this object
    const boost::mpi::communicator &world () const {return world_;}

    protected :

    size_type offset () const {return offset_;}

    int rank (size_type global_id) const
    {return static_cast<int>(global_id / this->size());}

    bool is_local (size_type global_id) const
    {return global_id >= offset_ && global_id < this->size() + offset_;}

    size_type local_id (size_type global_id) const
    {return global_id - this->size() * rank(global_id);}

    private :

    boost::mpi::communicator world_;
    size_type offset_;
    int copy_tag_;
    std::vector<size_type> copy_from_;
    std::vector<std::pair<int, size_type> > copy_recv_;
    std::vector<std::pair<int, size_type> > copy_send_;
}; // class StateMPI

} // namespace vsmc

#endif // VSMC_MPI_BACKEND_MPI_HPP
