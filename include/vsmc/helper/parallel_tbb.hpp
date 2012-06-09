#ifndef VSMC_HELPER_PARALLEL_TBB_HPP
#define VSMC_HELPER_PARALLEL_TBB_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/sequential.hpp>
#include <tbb/tbb.h>

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup Helper
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateTBB : public StateSeq<Dim, T>, public StateTBBTrait
{
    public :

    typedef typename StateBase<Dim, T>::size_type size_type;
    typedef T state_type;

    explicit StateTBB (size_type N) : StateSeq<Dim, T>(N), copy_(N) {}

    void copy (size_type from, size_type to)
    {
        copy_[to] = from;
    }

    void pre_resampling ()
    {
        for (size_type i = 0; i != this->size(); ++i)
            copy_[i] = i;
    }

    void post_resampling ()
    {
        tbb::parallel_for(tbb::blocked_range<size_type>(0, this->size()),
                work_(this, copy_.data()));
    }

    private :

    std::vector<size_type> copy_;

    class work_
    {
        public :

        work_ (StateTBB<Dim, T> *state, const size_type *from) :
            state_(state), from_(from) {}

        void operator () (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                size_type from = from_[i];
                if (from != i)
                    state_->state().col(i) = state_->state().col(from);
            }
        }

        private :

        StateTBB<Dim, T> *const state_;
        const size_type *const from_;
    }; // class work_
}; // class StateTBB

/// \brief Sampler<T>::init_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T>
class InitializeTBB : public InitializeSeq<T>, public InitializeTBBTrait
{
    public :

    typedef typename Particle<T>::size_type size_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        accept_.resize(particle.size());
        tbb::parallel_for(tbb::blocked_range<size_type>(0, particle.size()),
                work_(this, &particle, accept_.data()));
        this->post_processor(particle);

        return accept_.sum();
    }

    private :

    Eigen::Matrix<unsigned, Eigen::Dynamic, 1> accept_;

    class work_
    {
        public :

        work_ (InitializeTBB<T> *init,
                Particle<T> *particle, unsigned *accept) :
            init_(init), particle_(particle), accept_(accept) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                accept_[i] = init_->initialize_state(
                        SingleParticle<T>(i, particle_));
            }
        }

        private :

        InitializeTBB<T> *const init_;
        Particle<T> *const particle_;
        unsigned *const accept_;
    }; // class work_
}; // class InitializeTBB

/// \brief Sampler<T>::move_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T>
class MoveTBB : public MoveSeq<T>, public MoveTBBTrait
{
    public :

    typedef typename Particle<T>::size_type size_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        this->pre_processor(iter, particle);
        accept_.resize(particle.size());
        tbb::parallel_for(tbb::blocked_range<size_type>(0, particle.size()),
                work_(this, iter, &particle, accept_.data()));
        this->post_processor(iter, particle);

        return accept_.sum();
    }

    private :

    Eigen::Matrix<unsigned, Eigen::Dynamic, 1> accept_;

    class work_
    {
        public :

        work_ (MoveTBB<T> *move, unsigned iter,
                Particle<T> *particle, unsigned *accept) :
            move_(move), iter_(iter), particle_(particle), accept_(accept) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                accept_[i] = move_->move_state(iter_,
                        SingleParticle<T>(i, particle_));
            }
        }

        private :

        MoveTBB<T> *const move_;
        const unsigned iter_;
        Particle<T> *const particle_;
        unsigned *const accept_;
    }; // class work_
}; // class MoveTBB

/// \brief Monitor<T>::eval_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
/// \tparam Dim The dimension of the monitor
template <typename T, unsigned Dim>
class MonitorTBB : public MonitorSeq<T, Dim>, public MonitorTBBTrait
{
    public :

    typedef typename Particle<T>::size_type size_type;

    void operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, particle.size()),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);
    }

    private :

    class work_
    {
        public :

        work_ (MonitorTBB<T, Dim> *monitor, unsigned iter,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                monitor_->monitor_state(iter_,
                        ConstSingleParticle<T>(i, particle_), res_ + i * Dim);
            }
        }

        private :

        MonitorTBB<T, Dim> *const monitor_;
        const unsigned iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // class MonitorTBB

/// \brief Path<T>::eval_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T>
class PathTBB : public PathSeq<T>, public PathTBBTrait
{
    public :

    typedef typename Particle<T>::size_type size_type;

    double operator() (unsigned iter, const Particle<T> &particle,
            double *res)
    {
        this->pre_processor(iter, particle);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, particle.size()),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->width_state(iter, particle);
    }

    private :

    class work_
    {
        public :

        work_ (PathTBB<T> *path, unsigned iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                res_[i] = path_->path_state(iter_,
                        ConstSingleParticle<T>(i, particle_));
            }
        }

        private :

        PathTBB<T> *const path_;
        const unsigned iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // PathTBB

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_TBB_HPP
