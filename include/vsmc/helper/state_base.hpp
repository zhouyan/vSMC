#ifndef VSMC_HELPER_STATE_BASE_HPP
#define VSMC_HELPER_STATE_BASE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup Helper
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateBase : public internal::StateBaseTag, public internal::Types
{
    public :

    /// The type of state parameters
    typedef T state_type;

    /// The type of the matrix of states
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_mat_type;

    /// \brief Construct a StateBase object with given number of particles
    ///
    /// \param N The number of particles
    explicit StateBase (size_type N) : size_(N), state_(Dim, N) {}

    virtual ~StateBase () {}

    /// \brief The dimension of the problem
    ///
    /// \return The dimension of the parameter vector
    static unsigned dim ()
    {
        return Dim;
    }

    /// \brief The number of particles
    ///
    /// \return The number of particles in the current particle set
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

    /// \brief Read only access to a signle particle state
    ///
    /// \param id The position of the particle
    /// \param pos The position of the parameter in the state array
    ///
    /// \return A const reference to the parameter at position pos of the
    /// states array of the particle at position id
    const state_type &state (size_type id, unsigned pos) const
    {
        return state_(pos, id);
    }

    /// \brief Read and write access to the array of a single particle states
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A pointer to the array of states of the particle at position id
    state_type *state (size_type id)
    {
        return state_.col(id).data();
    }

    /// \brief Read only access to the array of a single particle states
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A const pointer to the array of states of the particle at
    /// position id
    const state_type *state (size_type id) const
    {
        return state_.col(id).data();
    }

    /// \brief Read and write access to the matrix of all particle states
    ///
    /// \return A reference to the states matrix
    ///
    /// \note state()(pos, id) == state(id, pos). Best avoid this feature
    state_mat_type &state ()
    {
        return state_;
    }

    /// \brief Read only access to the matrix of all particle states
    ///
    /// \return A const reference to the states matrix
    ///
    /// \note state()(pos, id) == state(id, pos). Best avoid this feature
    const state_mat_type &state () const
    {
        return state_;
    }

    /// \brief The copy method used by the Sampler
    ///
    /// \param from The index of particle whose state to be copied
    /// \param to The index of particle to which new state to be written
    virtual void copy (size_type from, size_type to)
    {
        state_.col(to) = state_.col(from);
    }

    private :

    size_type size_;
    state_mat_type state_;
}; // class StateBase

} // namespace vsmc

#endif // VSMC_HELPER_STATE_BASE_HPP
