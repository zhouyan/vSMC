#ifndef V_SMC_HELPER_PARALLEL_TBB_HPP
#define V_SMC_HELPER_PARALLEL_TBB_HPP

#include <Eigen/Dense>
#include <tbb/tbb.h>
#include <vSMC/internal/config.hpp>
#include <vSMC/helper/sequential.hpp>

namespace vSMC {

/// \brief Particle type class for helping implementing SMC using TBB
///
/// \note The template parameter has to be type StateTBB or its derived class.
/// \sa vSMC::StateSeq
template <unsigned Dim, typename T = double>
class StateTBB : public StateSeq<Dim, T>
{
    public :

    /// \brief Construct a StateTBB object with given number of particles
    ///
    /// \param N The number of particles
    StateTBB (std::size_t N) : StateSeq<Dim, T>(N) {}
}; // class StateTBB

/// \brief Sampler::init_type class for helping implementing SMC using TBB
///
/// \note The template parameter has to be type StateTBB or its derived class.
/// \sa vSMC::InitializeSeq
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
    std::size_t operator() (Particle<T> &particle, void *param)
    {
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        weight_.resize(particle.size());
        accept_.resize(particle.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, particle.size()),
                Worker_(this, &particle, particle.value().state(),
                    weight_.data(), accept_.data()));

        particle.set_log_weight(weight_.data());
        std::size_t accept = 0;
        for (std::size_t i = 0; i != particle.size(); ++i)
            accept += accept_[i];
        this->post_processor(particle);

        return accept;
    }

    private :

    Eigen::VectorXd weight_;
    Eigen::VectorXi accept_;

    class Worker_
    {
        public :

        Worker_ (InitializeTBB<T> *init,
                Particle<T> *particle, typename T::value_type *state,
                double *weight, int *accept) :
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
        int *const accept_;
    }; // class Woker_
}; // class InitializeTBB

/// \brief Sampler::move_type class for helping implementing SMC using TBB
///
/// \note The template parameter has to be type StateTBB or its derived class.
/// \sa vSMC::MoveSeq
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
    std::size_t operator () (std::size_t iter, Particle<T> &particle)
    {
        this->pre_processor(iter, particle);
        weight_.resize(particle.size());
        accept_.resize(particle.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, particle.size()),
                Worker_(this, iter, &particle, particle.value().state(),
                    weight_.data(), accept_.data()));
        MoveSeq<T>::set_weight(this->weight_action(), particle,
                weight_.data());
        std::size_t accept = 0;
        for (std::size_t i = 0; i != particle.size(); ++i)
            accept += accept_[i];
        this->post_processor(iter, particle);

        return accept;
    }

    private :

    Eigen::VectorXd weight_;
    Eigen::VectorXi accept_;

    class Worker_
    {
        public :

        Worker_ (MoveTBB<T> *move, std::size_t iter,
                Particle<T> *particle, typename T::value_type *state,
                double *weight, int *accept) :
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
        int *const accept_;
    }; // class Woker_
}; // class MoveTBB

/// \brief Monitor::integral_type class for helping implementing SMC using TBB
///
/// \note The template parameter has to be type StateTBB or its derived class.
/// \sa vSMC::MonitorSeq
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
    void operator () (std::size_t iter, Particle<T> &particle, double *res)
    {
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, particle.size()),
                Worker_(this, iter, &particle, particle.value().state(), res));
    }

    private :

    class Worker_
    {
        public :

        Worker_ (MonitorTBB<T, Dim> *monitor, std::size_t iter,
                Particle<T> *particle, typename T::value_type *state,
                double *res) :
            monitor_(monitor), iter_(iter), particle_(particle), state_(state),
            res_(res) {}

        void operator () (const tbb::blocked_range<std::size_t> &range) const
        {
            for (std::size_t i = range.begin(); i != range.end(); ++i) {
                monitor_->monitor_state(i, iter_, state_ + T::dim() * i,
                        *particle_, res_ + i * Dim);
            }
        }

        private :

        MonitorTBB<T, Dim> *const monitor_;
        const std::size_t iter_;
        Particle<T> *const particle_;
        typename T::value_type *const state_;
        double *const res_;
    }; // class Worker_
}; // class MonitorTBB

/// \brief Path::integral_type class for helping implementing SMC using TBB
///
/// \note The template parameter has to be type StateTBB or its derived class.
/// \sa vSMC::PathSeq
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
    double operator () (std::size_t iter, Particle<T> &particle, double *res)
    {
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, particle.size()),
                Worker_(this, iter, &particle, particle.value().state(), res));

        return this->width_state(iter, particle);
    }

    private :

    class Worker_
    {
        public :

        Worker_ (PathTBB<T> *path, std::size_t iter,
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
    }; // class Worker_
}; // PathTBB

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_TBB_HPP
