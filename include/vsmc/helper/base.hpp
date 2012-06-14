#ifndef VSMC_HELPER_BASE_HPP
#define VSMC_HELPER_BASE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/single_particle.hpp>

namespace vsmc { namespace internal {

/// \brief Particle::value_type subtype
/// \ingroup Helper
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateBase
{
    public :

    /// The type of the number of particles
    typedef VSMC_SIZE_TYPE size_type;

    /// The type of state parameters
    typedef T state_type;

    /// The type of the matrix of states
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_mat_type;

    /// Construct a StateBase object with given number of particles
    explicit StateBase (size_type N) : size_(N), state_(Dim, N) {}

    virtual ~StateBase () {}

    /// The dimension of the problem
    static unsigned dim ()
    {
        return Dim;
    }

    /// The number of particles
    size_type size () const
    {
        return size_;
    }

    /// \brief Read and write access to a signle particle state
    ///
    /// \param id The position of the particle
    /// \param pos The position of the parameter in the state array
    ///
    /// \return A reference to the parameter at position pos of the states
    /// array of the particle at position id
    state_type &state (size_type id, unsigned pos)
    {
        return state_(pos, id);
    }

    /// Read only access to a signle particle state
    const state_type &state (size_type id, unsigned pos) const
    {
        return state_(pos, id);
    }

    /// \brief Read and write access to the array of a single particle states
    ///
    /// \param id The position of the particle, 0 to size() - 1
    state_type *state (size_type id)
    {
        return state_.col(id).data();
    }

    /// Read only access to the array of a single particle states
    const state_type *state (size_type id) const
    {
        return state_.col(id).data();
    }

    /// \brief Read and write access to the matrix of all particle states
    ///
    /// \note The matrix is of column marjor as it's the default of Eigen.
    /// Therefore state()(pos, id) == state(id, pos). Best avoid this feature.
    state_mat_type &state ()
    {
        return state_;
    }

    /// Read only access to the matrix of all particle states
    const state_mat_type &state () const
    {
        return state_;
    }

    private :

    size_type size_;
    state_mat_type state_;
}; // class StateBase

template <typename T, typename Derived>
class MoveBase
{

    public :

    unsigned move_state (unsigned iter, SingleParticle<T> part)
    {
        return move_state<Derived>(iter, part, 0);
    }

    void post_processor (unsigned iter, Particle<T> &particle)
    {
        post_processor<Derived>(iter, particle, 0);
    }

    void pre_processor (unsigned iter, Particle<T> &particle)
    {
        pre_processor<Derived>(iter, particle, 0);
    }

    private :

    template <typename D, unsigned (D::*)(unsigned, SingleParticle<T>)>
    class move_state_sfinae_ {};

    template <typename D, void (D::*)(unsigned, Particle<T> &)>
    class processor_sfinae_ {};

    template <typename D>
    unsigned move_state (unsigned iter, SingleParticle<T> part,
            move_state_sfinae_<D, &D::move_state> *)
    {
        return static_cast<Derived *>(this)->move_state(iter, part);
    }

    template <typename D>
    unsigned move_state (unsigned, SingleParticle<T>, ...)
    {
        return 0;
    }

    template <typename D>
    void pre_processor (unsigned iter, Particle<T> &particle,
            processor_sfinae_<D, &D::pre_processor> *)
    {
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void pre_processor (unsigned iter, Particle<T> &particle, ...) {}

    template <typename D>
    void post_processor (unsigned iter, Particle<T> &particle,
            processor_sfinae_<D, &D::post_processor> *)
    {
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    template <typename D>
    void post_processor (unsigned iter, Particle<T> &particle, ...) {}
}; // class MoveBase

template <typename T>
class MoveBase<T, internal::VirtualBase>
{
    public :

    virtual unsigned move_state (unsigned iter, SingleParticle<T> part)
    {
        return 0;
    }

    virtual void post_processor (unsigned iter, Particle<T> &particle) {}
    virtual void pre_processor (unsigned iter, Particle<T> &particle) {}
}; // class MoveBase<T, internal::VirtualBase>

} } // namespace vsmc::internal

#endif // VSMC_HELPER_BASE_HPP
