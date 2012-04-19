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

    /// The type of parameters
    typedef T value_type;

    /// \brief Construct a StateBase object with given number of particles
    ///
    /// \param N The number of particles
    explicit StateBase (std::size_t N) : size_(N), state_(Dim, N) {}

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
    std::size_t size () const
    {
        return size_;
    }

    /// \brief Read and write access to the array of a single particle states
    ///
    /// \return A pointer to the states of a single particle
    T *state (std::size_t n)
    {
        return state_.col(n).data();
    }

    /// \brief Read only access to the array of a single particle states
    ///
    /// \return A const pointer to the states of a single array particle
    const T *state (std::size_t n) const
    {
        return state_.col(n).data();
    }

    /// \brief Read and write access to the array of all particle states
    ///
    /// \return A pointer to the states of all particles
    ///
    /// \note The array is of row major order. In other words, it is ordered
    /// as the first Dim elements are the states of first particle, the next
    /// Dim elements are the states of the second particle, and so on.
    T *state ()
    {
        return state_.data();
    }

    /// \brief Read only access to the array of all particle states
    ///
    /// \return A const pointer to the states of all particles
    const T *state () const
    {
        return state_.data();
    }

    /// \brief The copy method used by the Sampler
    ///
    /// \param from The index of particle whose state to be copied
    /// \param to The index of particle to which new state to be written
    void copy (std::size_t from, std::size_t to)
    {
	state_.col(to) = state_.col(from);
    }

    private :

    std::size_t size_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_;
}; // class StateBase

} // namespace vSMC

#endif // V_SMC_HELPER_STATE_BASE_HPP
