//============================================================================
// vSMC/include/vsmc/mpi/backend_mpi.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_MPI_BACKEND_MPI_HPP
#define VSMC_MPI_BACKEND_MPI_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/weight_set.hpp>
#include <vsmc/mpi/mpi_manager.hpp>
#include <vsmc/utility/aligned_memory.hpp>

#define VSMC_RUNTIME_ASSERT_MPI_BACKEND_MPI_COPY_SIZE_MISMATCH \
    VSMC_RUNTIME_ASSERT((N == global_size_),                                 \
            ("**StateMPI::copy** SIZE MISMATCH"))

namespace vsmc {

/// \brief Default MPI manager ID
/// \ingroup MPI
struct MPIDefault;

template <typename, typename = MPIDefault> class StateMPI;
template <typename = WeightSet, typename = MPIDefault> class WeightSetMPI;

/// \brief Particle::weight_set_type subtype using MPI
/// \ingroup MPI
template <typename WeightSetBase, typename ID>
class WeightSetMPI : public WeightSetBase
{
    public :

    typedef typename WeightSetBase::size_type size_type;

    explicit WeightSetMPI (size_type N) :
        WeightSetBase(N), world_(MPICommunicator<ID>::instance().get(),
                ::boost::mpi::comm_duplicate),
        internal_barrier_(true), resample_size_(0)
    {
        ::boost::mpi::all_reduce(world_, N, resample_size_,
                std::plus<size_type>());
        this->set_ess(static_cast<double>(resample_size_));
        barrier();
    }

    size_type resample_size () const {return resample_size_;}

    void read_resample_weight (double *first) const
    {
        barrier();
        gather_resample_weight();
        if (world_.rank() == 0) {
            const std::size_t S = static_cast<std::size_t>(world_.size());
            for (std::size_t r = 0; r != S; ++r) {
                first = std::copy(weight_all_[r].begin(), weight_all_[r].end(),
                        first);
            }
        }
        barrier();
    }

    const double *resample_weight_data () const
    {
        resample_weight_.resize(resample_size_);
        read_resample_weight(&resample_weight_[0]);

        return world_.rank() == 0 ? &resample_weight_[0] : VSMC_NULLPTR;
    }

    /// \brief A duplicated MPI communicator for this weight set object
    const ::boost::mpi::communicator &world () const {return world_;}

    void barrier () const {if (internal_barrier_) world_.barrier();}

    void internal_barrier (bool use) {internal_barrier_ = use;}

    protected :

    void normalize_log_weight ()
    {
        barrier();

        const size_type N = static_cast<size_type>(this->size());
        double *const lwptr = this->mutable_log_weight_data();

        double lmax_weight = lwptr[0];
        for (size_type i = 0; i != N; ++i)
            if (lmax_weight < lwptr[i])
                lmax_weight = lwptr[i];
        double gmax_weight = 0;
        ::boost::mpi::all_reduce(world_, lmax_weight, gmax_weight,
                ::boost::mpi::maximum<double>());
        for (size_type i = 0; i != N; ++i)
            lwptr[i] -= gmax_weight;

        barrier();
    }

    void normalize_weight ()
    {
        barrier();

        const size_type N = static_cast<size_type>(this->size());
        double *const wptr = this->mutable_weight_data();

        double lcoeff = 0;
        for (size_type i = 0; i != N; ++i)
            lcoeff += wptr[i];
        double gcoeff = 0;
        ::boost::mpi::all_reduce(world_, lcoeff, gcoeff, std::plus<double>());
        gcoeff = 1 / gcoeff;
        for (size_type i = 0; i != N; ++i)
            wptr[i] *= gcoeff;

        double less = 0;
        for (size_type i = 0; i != N; ++i)
            less += wptr[i] * wptr[i];
        double gess = 0;
        ::boost::mpi::all_reduce(world_, less, gess, std::plus<double>());
        gess = 1 / gess;
        this->set_ess(gess);

        barrier();
    }

    double compute_ess (const double *first, bool use_log) const
    {
        using std::exp;

        barrier();

        const size_type N = static_cast<size_type>(this->size());
        std::vector<double, AlignedAllocator<double> > buffer(N);
        double *const bptr = &buffer[0];

        if (use_log) {
            const double *const lwptr = this->log_weight_data();
            for (size_type i = 0; i != N; ++i)
                bptr[i] = lwptr[i] + first[i];
            double lmax_weight = bptr[0];
            for (size_type i = 0; i != N; ++i)
                if (lmax_weight < bptr[i])
                    lmax_weight = bptr[i];
            double gmax_weight = 0;
            ::boost::mpi::all_reduce(world_, lmax_weight, gmax_weight,
                    ::boost::mpi::maximum<double>());
            for (size_type i = 0; i != N; ++i)
                bptr[i] -= gmax_weight;
            for (size_type i = 0; i != N; ++i)
                bptr[i] = exp(bptr[i]);
        } else {
            const double *const wptr = this->weight_data();
            for (size_type i = 0; i != N; ++i)
                bptr[i] = wptr[i] * first[i];
        }

        double lcoeff = 0;
        for (size_type i = 0; i != N; ++i)
            lcoeff += bptr[i];
        double gcoeff = 0;
        ::boost::mpi::all_reduce(world_, lcoeff, gcoeff, std::plus<double>());
        gcoeff = 1 / gcoeff;
        for (size_type i = 0; i != N; ++i)
            bptr[i] *= gcoeff;

        double less = 0;
        for (size_type i = 0; i != N; ++i)
            less += bptr[i] * bptr[i];
        double gess = 0;
        ::boost::mpi::all_reduce(world_, less, gess, std::plus<double>());
        gess = 1 / gess;

        barrier();

        return gess;
    }

    double compute_cess (const double *first, bool use_log) const
    {
        using std::exp;

        barrier();

        const size_type N = static_cast<size_type>(this->size());
        const double *bptr = first;
        const double *const wptr = this->weight_data();
        std::vector<double, AlignedAllocator<double> > buffer;
        if (use_log) {
            buffer.resize(N);
            double *const cptr = &buffer[0];
            for (size_type i = 0; i != N; ++i)
                cptr[i] = exp(first[i]);
            bptr = cptr;
        }

        double labove = 0;
        double lbelow = 0;
        for (size_type i = 0; i != N; ++i) {
            double wb = wptr[i] * bptr[i];
            labove += wb;
            lbelow += wb * bptr[i];
        }
        double gabove = 0;
        double gbelow = 0;
        ::boost::mpi::all_reduce(world_, labove, gabove, std::plus<double>());
        ::boost::mpi::all_reduce(world_, lbelow, gbelow, std::plus<double>());

        barrier();

        return gabove * gabove / gbelow;
    }

    private :

    ::boost::mpi::communicator world_;
    bool internal_barrier_;
    size_type resample_size_;
    mutable std::vector<double> resample_weight_;
    mutable std::vector<double> weight_;
    mutable std::vector<std::vector<double> > weight_all_;

    void gather_resample_weight () const
    {
        weight_.resize(this->size());
        this->read_weight(&weight_[0]);
        if (world_.rank() == 0)
            ::boost::mpi::gather(world_, weight_, weight_all_, 0);
        else
            ::boost::mpi::gather(world_, weight_, 0);
    }
}; // class WeightSetMPI

/// \brief Particle::value_type subtype using MPI
/// \ingroup MPI
template <typename BaseState, typename ID>
class StateMPI : public BaseState
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;
    typedef WeightSetMPI<
        typename traits::WeightSetTypeTrait<BaseState>::type, ID
        > weight_set_type;
    typedef ID id;

    explicit StateMPI (size_type N) :
        BaseState(N), world_(MPICommunicator<ID>::instance().get(),
                ::boost::mpi::comm_duplicate),
        internal_barrier_(true),
        offset_(0), global_size_(0), size_equal_(true),
        copy_tag_(::boost::mpi::environment::max_tag())
    {
        ::boost::mpi::all_gather(world_, N, size_all_);
        const std::size_t R = static_cast<std::size_t>(world_.rank());
        const std::size_t S = static_cast<std::size_t>(world_.size());
        for (std::size_t i = 0; i != R; ++i) {
            offset_ += size_all_[i];
            global_size_ += size_all_[i];
            size_equal_ = size_equal_ && N == size_all_[i];
        }
        for (std::size_t i = R; i != S; ++i) {
            global_size_ += size_all_[i];
            size_equal_ = size_equal_ && N == size_all_[i];
        }
        barrier();
    }

    /// \brief Copy particles
    ///
    /// \param N The number of particles on all nodes
    /// \param copy_from A vector of length `N`, for each particle with global
    /// id `to`, `copy_from[to]` is the global id of the particle it shall
    /// copy.
    ///
    /// \details
    /// The `BaseState` type is required to have the following members -
    /// `state_pack_type`: A type that used to pack state values. It shall be
    /// serializable. That is, a `state_pack_type` object is acceptable by
    /// `boost::mpi::communicator::send` etc. Both StateMatrix::state_pack_type
    /// and StateTuple::state_pack_type satisfy this requirement if their
    /// template type parameter types are serializable. For user defined types,
    /// see document of Boost.Serialize of how to serialize a class object.
    /// - `state_pack`
    /// ~~~{.cpp}
    /// state_pack_type state_pack (size_type id) const;
    /// ~~~
    /// Given a local particle id on this node, pack the state values into a
    /// `state_pack_type` object.
    /// - `state_unpack`
    /// ~~~{.cpp}
    /// void state_unpack (size_type id, const state_pack_type &pack);
    /// ~~~
    /// Given a local particle id and a `state_pack_type` object, unpack it
    /// into the given position on this node. If C++11 rvalue reference is
    /// supported, then an rvalue version of this function can be defined to
    /// improved the performance as `pack` will be passed as an rvalue. e.g.,
    /// ~~~{.cpp}
    /// void state_unpack (size_type id, state_pack_type &&pack);
    /// ~~~
    /// As usual, if `state_pack_type` needs to explicitly define move
    /// constructor or assignment operator, care shall be taken to make sure
    /// that after the move `pack` is still a valid, assignable object.
    ///
    /// In vSMC, the resampling algorithms generate the number of replications
    /// of each particle. Particles with replication zero need to copy other
    /// particles. The vector of the number of replications is transfered to
    /// `copy_from` by Particle::resample, and it is generated in such a way
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
    /// `this->state_pack(id)` and `this->state_unpack(id, pack) is not too
    /// expansive, and inter-node copy is rare anyway. If this is not the case,
    /// then it can be a performance bottle neck.
    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_MPI_BACKEND_MPI_COPY_SIZE_MISMATCH;

        copy_pre_processor_dispatch(
                typename has_copy_pre_processor_<BaseState>::type());

        barrier();
        copy_from_.resize(N);
        if (world_.rank() == 0)
            std::copy(copy_from, copy_from + N, copy_from_.begin());
        ::boost::mpi::broadcast(world_, copy_from_, 0);
        copy_this_node(N, copy_from_.begin(), copy_recv_, copy_send_);
        barrier();
        copy_inter_node(copy_recv_, copy_send_);
        barrier();

        copy_post_processor_dispatch(
                typename has_copy_post_processor_<BaseState>::type());
    }

    /// \brief History of number of particles send from this node during copy
    const std::vector<std::size_t> &copy_send_num () const {return send_num_;}

    /// \brief A duplicated MPI communicator for this state value object
    const ::boost::mpi::communicator &world () const {return world_;}

    void barrier () const {if (internal_barrier_) world_.barrier();}

    void internal_barrier (bool use) {internal_barrier_ = use;}

    /// \brief The number of particles on all nodes
    size_type global_size () const {return global_size_;}

    /// \brief The number of particles on nodes with ranks less than the rank
    /// of this node.
    size_type offset () const {return offset_;}

    /// \brief Given a global particle id return the rank of the node it
    /// belongs
    int rank (size_type global_id) const
    {
        if (size_equal_)
            return static_cast<int>(global_id / this->size());

        std::size_t r = 0;
        size_type g = size_all_[0];
        while (g <= global_id) {
            ++r;
            g += size_all_[r];
        }

        return static_cast<int>(r);
    }

    /// \brief Given a global particle id check if it is on this `node`
    bool is_local (size_type global_id) const
    {return global_id >= offset_ && global_id < this->size() + offset_;}

    /// \brief Transfer a global particle id into a local particle id
    /// (possibly not on this node, use `rank` to get the rank of its node)
    size_type local_id (size_type global_id) const
    {
        if (size_equal_) {
            return global_id -
                this->size() * static_cast<size_type>(rank(global_id));
        }

        std::size_t r = 0;
        size_type g = size_all_[0];
        while (g <= global_id) {
            ++r;
            g += size_all_[r];
        }
        g -= size_all_[r];

        return global_id - g;
    }

    /// \brief Transfer a local particle id *on this node* into a global
    /// particle id
    size_type global_id (size_type local_id) const {return local_id + offset_;}

    protected :

    /// \brief The MPI recv/send tag used by `copy_inter_node`
    int copy_tag () const {return copy_tag_;}

    /// \brief Perform local copy
    ///
    /// \param N The number of particles on all nodes
    /// \param copy_from_first The beginning of the copy_from vector
    /// \param copy_recv All particles that shall be received at this node
    /// \param copy_send All particles that shall be send from this node
    ///
    /// \details
    /// `copy_from_first` can be a one-pass input iterator used to access a
    /// vector of size `N`, say `copy_from`.
    /// For each `to` in the range `0` to `N - 1`
    /// - If both `to` and `from = copy_from[to]` are particles on this node,
    /// use `BaseState::copy` to copy the parties. Otherwise,
    /// - If `to` is a particle on this node, insert a pair into `copy_recv`,
    /// whose values are the rank of the node from which this node shall
    /// receive the particle and the particle id *on this node* where the
    /// particle received shall be unpacked. Otherwise,
    /// - If `from = copy_from[to]` is a particle on this node, insert a pair
    /// into `copy_send`, whose values are the rank of the node to which this
    /// node shall send the particle and the particle id *on this node* where
    /// the particle sent shall be packed. Otherwise do nothing.
    ///
    /// It is important the the vector accessed through `copy_from_first` is
    /// the same for all nodes. Otherwise the behavior is undefined.
    template <typename InputIter>
    void copy_this_node (size_type N, InputIter copy_from_first,
            std::vector<std::pair<int, size_type> > &copy_recv,
            std::vector<std::pair<int, size_type> > &copy_send)
    {
        using std::advance;

        int rank_this = world_.rank();

        copy_from_this_.resize(this->size());
        InputIter first = copy_from_first;
        advance(first, static_cast<typename std::iterator_traits<InputIter>::
                difference_type>(offset_));
        for (size_type to = 0; to != this->size(); ++to, ++first) {
            size_type from = *first;
            copy_from_this_[to] =
                rank_this == rank(from) ? local_id(from) : to;
        }
        BaseState::copy(this->size(), &copy_from_this_[0]);

        copy_recv.clear();
        copy_send.clear();
        for (size_type to = 0; to != N; ++to, ++copy_from_first) {
            size_type from = *copy_from_first;
            int rank_recv = rank(to);
            int rank_send = rank(from);
            size_type id_recv = local_id(to);
            size_type id_send = local_id(from);
            if (rank_this == rank_recv && rank_this == rank_send) {
                continue;
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
        send_num_.push_back(copy_send.size());
        int rank_this = world_.rank();
        for (int r = 0; r != world_.size(); ++r) {
            if (rank_this == r) {
                for (std::size_t i = 0; i != copy_recv.size(); ++i) {
                    world_.recv(copy_recv_[i].first, copy_tag_, pack_recv_);
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
                    this->state_unpack(copy_recv[i].second,
                            cxx11::move(pack_recv_));
#else
                    this->state_unpack(copy_recv[i].second, pack_recv_);
#endif
                }
            } else {
                for (std::size_t i = 0; i != copy_send.size(); ++i) {
                    if (copy_send_[i].first == r) {
                        pack_send_ = this->state_pack(copy_send[i].second);
                        world_.send(copy_send_[i].first, copy_tag_,
                                pack_send_);
                    }
                }
            }
        }
    }

    private :

    ::boost::mpi::communicator world_;
    bool internal_barrier_;
    size_type offset_;
    size_type global_size_;
    bool size_equal_;
    std::vector<size_type> size_all_;
    int copy_tag_;
    std::vector<size_type> copy_from_;
    std::vector<size_type> copy_from_this_;
    std::vector<std::pair<int, size_type> > copy_recv_;
    std::vector<std::pair<int, size_type> > copy_send_;
    typename BaseState::state_pack_type pack_recv_;
    typename BaseState::state_pack_type pack_send_;
    std::vector<std::size_t> send_num_;

    VSMC_DEFINE_METHOD_CHECKER(copy_pre_processor, void, ())
    VSMC_DEFINE_METHOD_CHECKER(copy_post_processor, void, ())

    void copy_pre_processor_dispatch (cxx11::true_type)
    {
        barrier();
        this->copy_pre_processor();
    }

    void copy_pre_processor_dispatch (cxx11::false_type) {}

    void copy_post_processor_dispatch (cxx11::true_type)
    {
        this->copy_post_processor();
        barrier();
    }

    void copy_post_processor_dispatch (cxx11::false_type) {}
}; // class StateMPI

} // namespace vsmc

#endif // VSMC_MPI_BACKEND_MPI_HPP
