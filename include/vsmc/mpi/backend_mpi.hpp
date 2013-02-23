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
    /// \param N The number of particles on all nodes
    /// \param copy_from A vector of length `N`, for each particle with global
    /// id `to`, `copy_from[to]` is the global id of the particle it shall
    /// copy.
    ///
    /// \details
    /// The `BaseState` type is required to have the following members
    /// - `state_pack_type`: A type that used to pack state values. It shall be
    /// serializable. That is, a `state_pack_type` object is acceptable by
    /// `boost::mpi::communicator::send` etc. Both
    /// `StateMatrix::state_pack_type` and `StateTuple::state_pack_type`
    /// satisfy this requirement if their template type parameter types are
    /// serializable. For user defined types, see document of Boost.Serialize
    /// of how to serialize a class object.
    /// - `state_pack`
    /// \code
    /// state_pack_type state_pack (size_type id) const;
    /// \endcode
    /// Given a local particle id on this node, pack the state values into a
    /// `state_pack_type` object.
    /// - `state_unpack`
    /// \code
    /// void state_unpack (size_type id, const state_pack_type &pack);
    /// \endcode
    /// Given a local particle id and a `state_pack_type` object, unpack it
    /// into the given position on this node.
    ///
    /// In vSMC, the resampling algorithms generate the number of replications
    /// of each particle. Particles with replication zero need to copy other
    /// particles. The vector of the number of replications is transfered to
    /// `copy_from` by `Particle::resample`, and it is generated in such a way
    /// that each particle will copy from somewhere close to itself. Therefore,
    /// transferring between nodes is minimized.
    ///
    /// This default implementation perform three stages of copy.
    /// - Stage one: Generate a local duplicate of `copy_from` on node `0` and
    /// broadcast it to all nodes.
    /// - Stage two: Perform local copy, copy those particles where the
    /// destination and source are both on this node. This is performed in
    /// parallel on each node.
    /// - Stage three: copy particles that need message passing between nodes.
    ///
    /// A derived class can override this `copy` method. For the following
    /// possible reasons,
    /// - Stage one is not needed or too expansive
    /// - Stage three is too expansive. The default implementation assumes
    /// `this->state_pack(id)` is not too expansive, and inter-node copy is
    /// rare anyway. If this is not the case, then it can be a performance
    /// bottle neck.
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

        copy_this_node(N, &copy_from_[0], copy_recv_, copy_send_);
        world_.barrier();

        copy_inter_node(copy_recv_, copy_send_);
        world_.barrier();
    }

    /// \brief A duplicated MPI communicator for this object
    const boost::mpi::communicator &world () const {return world_;}

    /// \brief The number of particles on all nodes
    size_type global_size () const
    {return this->size() * static_cast<size_type>(world_.size());}

    /// \brief The number of particles on nodes with ranks less than the rank
    /// of this node.
    size_type offset () const {return offset_;}

    /// \brief Given a global particle id return the rank of the node it
    /// belongs
    int rank (size_type global_id) const
    {return static_cast<int>(global_id / this->size());}

    /// \brief Given a global particle id check if it is on this `node`
    bool is_local (size_type global_id) const
    {return global_id >= offset_ && global_id < this->size() + offset_;}

    /// \brief Transfer a global particle id into a local particle id
    /// (possibly not on this node, use `rank` to get the rank of its node)
    size_type local_id (size_type global_id) const
    {return global_id - this->size() * rank(global_id);}

    /// \brief Transfer a local particle id *on this node* into a global
    /// particle id
    size_type global_id (size_type lcoal_id) const {return local_id + offset_;}

    protected :

    /// \brief The MPI recv/send tag used by `copy_inter_node`
    int copy_tag () const {return copy_tag_;}

    /// \brief Perform local copy
    ///
    /// \param N The number of particles on all nodes
    /// \param copy_from_first The first iterator of the copy_from vector
    /// \param copy_recv All particles that shall be received at this node
    /// \param copy_send All particles that shall be send from this node
    ///
    /// \details
    /// `copy_from_first` can be a one-pass input iterator used to access a
    /// vector of size `N`, say `copy_from`.
    /// For each `to` in the range `0` to `N - 1`
    /// - If both `to` and `from = copy_from[to]` are particles on this node,
    /// invoke `copy_particle` to copy the parties. Otherwise,
    /// - If `to` is a particle on this node, insert a pair into `copy_recv`,
    /// whose values are the rank of the node from which this node shall
    /// receive the particle and the particle id *on this node* where the
    /// particle received shall be unpacked. Otherwise,
    /// - If `from = copy_from[to]` is a particle on this node, insert a pair
    /// into `copy_send`, whose values are the rank of the node to which this
    /// node shall send the particle  and the particle id *on this node* where
    /// the particle sent shall be packed. Otherwise do nothing.
    ///
    /// \note
    /// It is important the the vector access through `copy_from_first` is the
    /// same for all nodes. Otherwise the behavior is undefined.
    template <typename InputIter>
    void copy_this_node (size_type N, InputIter copy_from_first,
            std::vector<std::pair<int, size_type> > &copy_recv,
            std::vector<std::pair<int, size_type> > &copy_send)
    {
        copy_recv.clear();
        copy_send.clear();
        int rank_this = world_.rank();
        for (size_type to = 0; to != N; ++to, ++copy_from_first) {
            size_type from = *copy_from_first;
            int rank_recv = rank(to);
            int rank_send = rank(from);
            size_type id_recv = local_id(to);
            size_type id_send = local_id(from);
            if (rank_this == rank_recv && rank_this == rank_send) {
                this->copy_particle(id_send, id_recv);
            } else if (rank_this == rank_recv) {
                copy_recv.push_back(std::make_pair(rank_send, id_recv));
            } else if (rank_this == rank_send) {
                copy_send.push_back(std::make_pair(rank_recv, id_send));
            }
        }
    }

    /// \brief Perform global copy
    ///
    /// \param copy_recv The output vector `copy_recv` from `copy_this_node`
    /// \param copy_send The output vector `copy_send` from `copy_this_node`
    void copy_inter_node (
            const std::vector<std::pair<int, size_type> > &copy_recv,
            const std::vector<std::pair<int, size_type> > &copy_send)
    {
        int rank_this = world_.rank();
        for (int r = 0; r != world_.size(); ++r) {
            if (rank_this == r) {
                for (std::size_t i = 0; i != copy_recv.size(); ++i) {
                    typename BaseState::state_pack_type pack;
                    world_.recv(copy_recv_[i].first, copy_tag_, pack);
                    this->state_unpack(copy_recv[i].second, pack);
                }
            } else {
                for (std::size_t i = 0; i != copy_send.size(); ++i) {
                    if (copy_send_[i].first == r) {
                        world_.send(copy_send_[i].first, copy_tag_,
                                this->state_pack(copy_send[i].second));
                    }
                }
            }
        }
    }

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
