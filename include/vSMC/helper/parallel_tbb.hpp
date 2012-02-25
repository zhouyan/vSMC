#ifndef V_SMC_HELPER_PARALLEL_TBB_HPP
#define V_SMC_HELPER_PARALLEL_TBB_HPP

#include <tbb/tbb.h>
#include <vSMC/helper/sequential.hpp>

namespace vSMC { namespace internal {

template <typename T> class InitializeTBBApply;
template <typename T> class MoveTBBApply;
template <typename T> class MonitorTBBApply;
template <typename T> class PathTBBApply;

} // namespace vSMC::internal


/// \brief Particle type for helping implementing SMC using TBB
///
/// StateTBB or its derived class can be used as the template argument of
/// Particle. It targets the particular problems where the parameters to be
/// sampled can be viewed as a vector of dimension Dim and type T.
template <int Dim, typename T = double>
class StateTBB : public StateSeq<Dim, T>
{
    public :

    /// \brief Construct a StateTBB object with given number of particles
    ///
    /// \param N The number of particles
    StateTBB (std::size_t N) : StateSeq(N), size_(N) {}
}; // class StateTBB

template <typename T>
class InitializeTBB
{
    public :

    InitializeTBB () {}

    InitializeTBB (const InitializeTBB<T> &init) {}

    InitializeTBB<T> & operator= (const InitializeTBB<T> &init) {return *this;}

    /// \brief Operator called by Sampler for initialize the particle set
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param param Additional parameters
    ///
    /// \return Accept count
    virtual std::size_t operator() (Particle<T> &particle, void *param)
    {
        initialize_param(particle, param);

        weight_.resize(particle.size());
        accept_.resize(particle.size());
        std::size_t accept = 0;

        particle.set_log_weight(weight_);

        return accept;
    }

    /// \brief Initialize a single particle
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param state The array contains the states of a single particle
    /// \param weight The log weight of the particle
    ///
    /// \return Accept count, normally should be zero or one
    virtual int initialize_state (const Particle<T> &particle,
            typename T::value_type *state, double &weight) = 0;

    /// \brief Initialize the Particle set
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param param Additional parameter passed by Sampler
    virtual void initialize_param (Particle<T> &particle, void *param) {};

    friend internal::InitializeTBBApply

    private :

    vDist::tool::Buffer<double> weight_;
    vDist::tool::Buffer<int> accept_;

    int &accept (std::size_t n)
    {
        return accept[n];
    }

    int accept (std::size_t n) const
    {
        return accept[n];
    }

    double &weight (std::size_t n)
    {
        return weight[n];
    }

    double &weight (std::size_t n) const
    {
        return weight[n];
    }
}; // class InitializeTBB

namespace internal {

template <typename T>
class InitializeTBBApply
{
    public :

    InitializeTBBApply (InitializeTBB<T> *init, const Particle<T> *particle) :
        init_(init), particle_(particle) {}

    void operator () (const tbb::blocked_range<std::size_t> &range) const
    {
        for (std::size_t i = range.begin(); i != range.end(); ++i) {
            init_->accept(i) = init_->initialize_state(*particle_,
                    particle_->value().state(i), init_->weight(i));
        }
    }

    private :

    InitializeTBB<T> *const init_; 
    const Particle<T> *const particle_;
}; // class InitializeTBBApply

} // namespace vSMC::internal

template <typename T>
class MoveTBB
{
}; // class MoveTBB

template <typename T>
class MonitorTBB
{
}; // class MonitorTBB

template <typename T>
class PathTBB
{
}; // PathTBB

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_TBB_HPP
