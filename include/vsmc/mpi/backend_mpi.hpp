//============================================================================
// vSMC/include/vsmc/mpi/backend_mpi.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#include <vsmc/mpi/internal/common.hpp>
#include <vsmc/core/weight.hpp>
#include <vsmc/mpi/mpi_datatype.hpp>
#include <vsmc/mpi/mpi_manager.hpp>

#define VSMC_RUNTIME_ASSERT_MPI_BACKEND_MPI_COPY_SIZE_MISMATCH                \
    VSMC_RUNTIME_ASSERT(                                                      \
        (N == global_size_), "**StateMPI::copy** SIZE MISMATCH")

namespace vsmc
{

/// \brief Particle::weight_type subtype using MPI
/// \ingroup MPI
template <typename WeightBase, typename ID = MPIDefault>
class WeightMPI : public WeightBase
{
    public:
    using size_type = SizeType<WeightBase>;
    using mpi_id = ID;

    explicit WeightMPI(size_type N)
        : WeightBase(N)
        , world_(MPICommunicator<ID>::instance().get(),
              ::boost::mpi::comm_duplicate)
        , resample_size_(0)
    {
        ::boost::mpi::all_reduce(
            world_, N, resample_size_, std::plus<size_type>());
    }

    /// \brief A duplicated MPI communicator for this weight set object
    const ::boost::mpi::communicator &world() const { return world_; }

    size_type resample_size() const { return resample_size_; }

    void read_resample_weight(double *first) const
    {
        gather_resample_weight();
        if (world_.rank() == 0) {
            const std::size_t S = static_cast<std::size_t>(world_.size());
            for (std::size_t r = 0; r != S; ++r) {
                first = std::copy(
                    weight_all_[r].begin(), weight_all_[r].end(), first);
            }
        }
    }

    const double *resample_data() const
    {
        resample_weight_.resize(resample_size_);
        read_resample_weight(resample_weight_.data());

        return world_.rank() == 0 ? resample_weight_.data() : nullptr;
    }

    private:
    ::boost::mpi::communicator world_;
    size_type resample_size_;
    mutable std::vector<double> resample_weight_;
    mutable std::vector<double> weight_;
    mutable std::vector<std::vector<double>> weight_all_;

    void gather_resample_weight() const
    {
        weight_.resize(this->size());
        this->read_weight(weight_.data());
        if (world_.rank() == 0)
            ::boost::mpi::gather(world_, weight_, weight_all_, 0);
        else
            ::boost::mpi::gather(world_, weight_, 0);
    }

    double get_ess() const
    {
        const std::size_t N = static_cast<std::size_t>(this->size());
        const double *const wptr = this->data();

        double less = dot(N, wptr, 1, wptr, 1);
        double gess = 0;
        ::boost::mpi::all_reduce(world_, less, gess, std::plus<double>());

        return 1 / gess;
    }

    void normalize()
    {
        const std::size_t N = static_cast<std::size_t>(this->size());
        double *const wptr = this->mutable_data();

        double lcoeff = std::accumulate(wptr, wptr + N, 0.0);
        double gcoeff = 0;
        ::boost::mpi::all_reduce(world_, lcoeff, gcoeff, std::plus<double>());
        gcoeff = 1 / gcoeff;
        mul(N, gcoeff, wptr, wptr);
    }

    void normalize_log()
    {
        const std::size_t N = static_cast<std::size_t>(this->size());
        double *const wptr = this->mutable_data();

        double lmax_weight = *(std::max_element(wptr, wptr + N));
        double gmax_weight = 0;
        ::boost::mpi::all_reduce(
            world_, lmax_weight, gmax_weight, ::boost::mpi::maximum<double>());
        for (std::size_t i = 0; i != N; ++i)
            wptr[i] -= gmax_weight;
    }
}; // class WeightMPI

/// \brief Particle::value_type subtype using MPI
/// \ingroup MPI
template <typename StateBase, typename ID = MPIDefault>
class StateMPI : public StateBase
{
    public:
    using size_type = SizeType<StateBase>;
    using weight_type = WeightMPI<WeightType<StateBase>, ID>;
    using mpi_id = ID;

    explicit StateMPI(size_type N)
        : StateBase(N)
        , world_(MPICommunicator<ID>::instance().get(),
              ::boost::mpi::comm_duplicate)
        , offset_(0)
        , global_size_(0)
        , size_equal_(true)
        , copy_tag_(::boost::mpi::environment::max_tag())
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
    }

    /// \brief Copy particles
    ///
    /// \param N The number of particles on all nodes
    /// \param src_idx A vector of length `N`, for each particle with global
    /// id `dst`, `src_idx[dst]` is the global id of the particle it shall
    /// copy.
    template <typename IntType>
    void copy(size_type N, const IntType *src_idx)
    {
        VSMC_RUNTIME_ASSERT_MPI_BACKEND_MPI_COPY_SIZE_MISMATCH;

        copy_pre_dispatch(has_copy_pre_<StateBase>());
        src_idx_.resize(N);
        if (world_.rank() == 0)
            std::copy(src_idx, src_idx + N, src_idx_.begin());
        ::boost::mpi::broadcast(world_, src_idx_, 0);
        copy_this_node(N, src_idx_.data(), copy_recv_, copy_send_);
        copy_inter_node(copy_recv_, copy_send_);
        copy_post_dispatch(has_copy_post_<StateBase>());
    }

    /// \brief A duplicated MPI communicator for this state value object
    const ::boost::mpi::communicator &world() const { return world_; }

    /// \brief The number of particles on all nodes
    size_type global_size() const { return global_size_; }

    /// \brief The number of particles on nodes with ranks less than the rank
    /// of this node.
    size_type offset() const { return offset_; }

    /// \brief Given a global particle id return the rank of the node it
    /// belongs
    int rank(size_type global_id) const
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
    bool is_local(size_type global_id) const
    {
        return global_id >= offset_ && global_id < this->size() + offset_;
    }

    /// \brief Transfer a global particle id into a local particle id
    /// (possibly not on this node, use `rank` to get the rank of its node)
    size_type local_id(size_type global_id) const
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
    size_type global_id(size_type local_id) const
    {
        return local_id + offset_;
    }

    protected:
    /// \brief The MPI recv/send tag used by `copy_inter_node`
    int copy_tag() const { return copy_tag_; }

    /// \brief Perform local copy
    ///
    /// \param N The number of particles on all nodes
    /// \param src_idx The beginning of the src_idx vector
    /// \param copy_recv All particles that shall be received at this node
    /// \param copy_send All particles that shall be send from this node
    void copy_this_node(size_type N, const size_type *src_idx,
        std::vector<std::pair<int, size_type>> &copy_recv,
        std::vector<std::pair<int, size_type>> &copy_send)
    {
        using std::advance;

        const int rank_this = world_.rank();

        src_idx_this_.resize(this->size());
        const size_type *first = src_idx + offset_;
        for (size_type dst = 0; dst != this->size(); ++dst, ++first) {
            size_type src = *first;
            src_idx_this_[dst] = rank_this == rank(src) ? local_id(src) : dst;
        }
        StateBase::copy(this->size(), src_idx_this_.data());

        copy_recv.clear();
        copy_send.clear();
        for (size_type dst = 0; dst != N; ++dst) {
            size_type src = src_idx[dst];
            int rank_recv = rank(dst);
            int rank_send = rank(src);
            size_type id_recv = local_id(dst);
            size_type id_send = local_id(src);
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
    void copy_inter_node(
        const std::vector<std::pair<int, size_type>> &copy_recv,
        const std::vector<std::pair<int, size_type>> &copy_send)
    {
        const int rank_this = world_.rank();
        for (int r = 0; r != world_.size(); ++r) {
            if (rank_this == r) {
                for (std::size_t i = 0; i != copy_recv.size(); ++i) {
                    world_.recv(copy_recv[i].first, copy_tag_, pack_recv_);
                    this->state_unpack(
                        copy_recv[i].second, std::move(pack_recv_));
                }
            } else {
                for (std::size_t i = 0; i != copy_send.size(); ++i) {
                    if (copy_send[i].first == r) {
                        pack_send_ = this->state_pack(copy_send[i].second);
                        world_.send(copy_send[i].first, copy_tag_, pack_send_);
                    }
                }
            }
        }
    }

    private:
    ::boost::mpi::communicator world_;
    size_type offset_;
    size_type global_size_;
    bool size_equal_;
    std::vector<size_type> size_all_;
    int copy_tag_;
    std::vector<size_type> src_idx_;
    std::vector<size_type> src_idx_this_;
    std::vector<std::pair<int, size_type>> copy_recv_;
    std::vector<std::pair<int, size_type>> copy_send_;
    typename StateBase::state_pack_type pack_recv_;
    typename StateBase::state_pack_type pack_send_;

    VSMC_DEFINE_METHOD_CHECKER(copy_pre, void, ())
    VSMC_DEFINE_METHOD_CHECKER(copy_post, void, ())

    void copy_pre_dispatch(std::true_type) { StateBase::copy_pre(); }
    void copy_pre_dispatch(std::false_type) {}
    void copy_post_dispatch(std::true_type) { StateBase::copy_post(); }
    void copy_post_dispatch(std::false_type) {}
}; // class StateMPI

} // namespace vsmc

#endif // VSMC_MPI_BACKEND_MPI_HPP
