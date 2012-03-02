#ifndef V_SMC_HELPER_PARALLEL_TBB_HPP
#define V_SMC_HELPER_PARALLEL_TBB_HPP

#include <tbb/tbb.h>
#include <vSMC/helper/sequential.hpp>

namespace vSMC {

template <typename T> class InitializeTBB;
template <typename T> class MoveTBB;
template <typename T, unsigned Dim> class MonitorTBB;
template <typename T> class PathTBB;

namespace internal {

template <typename T>
class InitializeTBBApply
{
    public :

    InitializeTBBApply (InitializeTBB<T> *init,
            Particle<T> *particle, typename T::value_type *state,
            double *weight, unsigned *accept) :
        init_(init), particle_(particle), state_(state),
        weight_(weight), accept_(accept) {}

    void operator () (const tbb::blocked_range<std::size_t> &range) const
    {
        for (std::size_t i = range.begin(); i != range.end(); ++i) {
            accept_[i] = init_->initialize_state(i, state_ + T::dim() * i,
                    weight_[i], *particle_, particle_->prng(i));
        }
    }

    private :

    InitializeTBB<T> *const init_; 
    Particle<T> *const particle_;
    typename T::value_type *const state_;
    double *const weight_;
    unsigned *const accept_;
}; // class InitializeTBBApply

template <typename T>
class MoveTBBApply
{
    public :

    MoveTBBApply (MoveTBB<T> *move, std::size_t iter,
            Particle<T> *particle, typename T::value_type *state,
            double *weight, unsigned *accept) :
        move_(move), iter_(iter), particle_(particle), state_(state),
        weight_(weight), accept_(accept) {}

    void operator () (const tbb::blocked_range<std::size_t> &range) const
    {
        for (std::size_t i = range.begin(); i != range.end(); ++i) {
            accept_[i] = move_->move_state(i, iter_, state_ + T::dim() * i,
                    weight_[i], *particle_, particle_->prng(i));
        }
    }

    private :

    MoveTBB<T> *const move_; 
    const std::size_t iter_;
    Particle<T> *const particle_;
    typename T::value_type *const state_;
    double *const weight_;
    unsigned *const accept_;
}; // class MoveTBBApply

template <typename T, unsigned Dim>
class MonitorTBBApply
{
    public :

    MonitorTBBApply (MonitorTBB<T, Dim> *monitor, std::size_t iter,
            Particle<T> *particle, typename T::value_type *state,
            double *res) :
        monitor_(monitor), iter_(iter), particle_(particle), state_(state),
        res_(res) {}

    void operator () (const tbb::blocked_range<std::size_t> &range) const
    {
        for (std::size_t i = range.begin(); i != range.end(); ++i) {
            monitor_->monitor_state(i, iter_, state_ + T::dim() * i,
                    *particle_, res_ + i * dim());
        }
    }

    static unsigned dim ()
    {
        return Dim;
    }

    private :

    MonitorTBB<T, Dim> *const monitor_;
    const std::size_t iter_;
    Particle<T> *const particle_;
    typename T::value_type *const state_;
    double *const res_;
}; // class MonitorTBBApply

template <typename T>
class PathTBBApply
{
    public :

    PathTBBApply (PathTBB<T> *path, std::size_t iter,
            Particle<T> *particle, typename T::value_type *state,
            double *res) :
        path_(path), iter_(iter), particle_(particle), state_(state),
        res_(res) {}

    void operator () (const tbb::blocked_range<std::size_t> &range) const
    {
        for (std::size_t i = range.begin(); i != range.end(); ++i) {
            res_[i] = path_->path_state(i, iter_, state_ + T::dim() * i,
                    *particle_);
        }
    }

    private :

    PathTBB<T> *const path_;
    const std::size_t iter_;
    Particle<T> *const particle_;
    typename T::value_type *const state_;
    double *const res_;
}; // class PathTBBApply

} // namespace vSMC::internal

/// \brief Particle type for helping implementing SMC using TBB
///
/// StateTBB or its derived class can be used as the template argument of
/// Particle. It targets the particular problems where the parameters to be
/// sampled can be viewed as a vector of dimension Dim and type T.
template <unsigned Dim, typename T = double>
class StateTBB : public StateSeq<Dim, T>
{
    public :

    /// \brief Construct a StateTBB object with given number of particles
    ///
    /// \param N The number of particles
    StateTBB (std::size_t N) : StateSeq<Dim, T>(N) {}
}; // class StateTBB

template <typename T>
class InitializeTBB : public InitializeSeq<T>
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
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        weight_.resize(particle.size());
        accept_.resize(particle.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, particle.size()),
                internal::InitializeTBBApply<T>(this, &particle,
                    particle.value().state(), weight_.data(), accept_.data()));

        particle.set_log_weight(weight_.data());
        std::size_t accept = 0;
        for (std::size_t i = 0; i != particle.size(); ++i)
            accept += accept_[i];
        this->post_processor(particle);

        return accept;
    }

    private :

    internal::Buffer<double> weight_;
    internal::Buffer<unsigned> accept_;
}; // class InitializeTBB

template <typename T>
class MoveTBB : public MoveSeq<T>
{
    public :

    MoveTBB () {}
    MoveTBB (const MoveTBB<T> &move) {}
    MoveTBB<T> & operator= (const MoveTBB<T> &move) {return *this;}

    /// \brief Operator called by Sampler for move the particle set
    ///
    /// \param iter The iteration number
    /// \param particle The Particle set passed by Sampler
    ///
    /// \return Accept count
    virtual std::size_t operator () (std::size_t iter, Particle<T> &particle)
    {
        this->pre_processor(iter, particle);
        weight_.resize(particle.size());
        accept_.resize(particle.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, particle.size()),
                internal::MoveTBBApply<T>(this, iter, &particle,
                    particle.value().state(), weight_.data(), accept_.data()));
        MoveSeq<T>::set_weight(this->weight_action(), particle,
                weight_.data());
        std::size_t accept = 0;
        for (std::size_t i = 0; i != particle.size(); ++i)
            accept += accept_[i];
        this->post_processor(iter, particle);

        return accept;
    }

    private :

    internal::Buffer<double> weight_;
    internal::Buffer<unsigned> accept_;
}; // class MoveTBB

template <typename T, unsigned Dim = 1>
class MonitorTBB : public MonitorSeq<T, Dim>
{
    public :

    /// \brief Operator called by Monitor to record Monte Carlo integration
    ///
    /// \param iter The iteration number
    /// \param particle The Particle set passed by Sampler
    /// \param [out] res The integrands. Sum(res * weight) is the Monte Carlo
    /// integration result.
    virtual void operator () (std::size_t iter, Particle<T> &particle,
            double *res)
    {
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, particle.size()),
                internal::MonitorTBBApply<T, Dim>(this, iter, &particle,
                    particle.value().state(), res));
    }
}; // class MonitorTBB

template <typename T>
class PathTBB : public PathSeq<T>
{
    public :

    /// \brief Operator called by Path to record path sampling integrands and
    /// widths
    ///
    /// \param iter The iteration number
    /// \param particle The particle set passed by Sampler
    /// \param [out] res The integrands. Sum(res * weight) is the path
    ///
    /// \return The width
    /// sampling integrand.
    virtual double operator () (std::size_t iter, Particle<T> &particle,
            double *res)
    {
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, particle.size()),
                internal::PathTBBApply<T>(this, iter, &particle,
                    particle.value().state(), res));

        return this->width_state(iter, particle);
    }
}; // PathTBB

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_TBB_HPP
