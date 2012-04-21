#ifndef V_SMC_HELPER_STATE_BASE_HPP
#define V_SMC_HELPER_STATE_BASE_HPP

#include <Eigen/Dense>
#include <vSMC/internal/config.hpp>

namespace vSMC {

/// \brief Particle type class for helping implementing SMC sequentially
///
/// StateBase or its derived class can be used as the template argument of
/// Particle. It targets the particular problems where the parameters to be
/// sampled can be viewed as a vector of dimension Dim and type T.
template <unsigned Dim, typename T = double>
class StateBase
{
    public :

    /// The type of state parameters
    typedef T state_type;

    /// The type of the matrix of states
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_mat_type;

    /// The type of the size of the particle set
    typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE size_type;

    /// \brief Construct a StateBase object with given number of particles
    ///
    /// \param N The number of particles
    explicit StateBase (size_type N) : size_(N), state_(Dim, N) {}

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

    /// \brief Read and write access to the array of a single particle states
    ///
    /// \return A pointer to the states of a single particle
    state_type *state (size_type n)
    {
        return state_.col(n).data();
    }

    /// \brief Read only access to the array of a single particle states
    ///
    /// \return A const pointer to the states of a single array particle
    const state_type *state (size_type n) const
    {
        return state_.col(n).data();
    }

    /// \brief Read and write access to the matrix of all particle states
    ///
    /// \return A reference to the states matrix
    state_mat_type &state ()
    {
        return state_;
    }

    /// \brief Read only access to the matrix of all particle states
    ///
    /// \return A const reference to the states matrix
    const state_mat_type &state () const
    {
        return state_;
    }

    /// \brief The copy method used by the Sampler
    ///
    /// \param from The index of particle whose state to be copied
    /// \param to The index of particle to which new state to be written
    void copy (size_type from, size_type to)
    {
        state_.col(to) = state_.col(from);
    }

    private :

    size_type size_;
    state_mat_type state_;
}; // class StateBase

} // namespace vSMC

#endif // V_SMC_HELPER_STATE_BASE_HPP
