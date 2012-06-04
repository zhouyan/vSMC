#ifndef V_SMC_HELPER_PARALLEL_TBB_HPP
#define V_SMC_HELPER_PARALLEL_TBB_HPP

#include <vSMC/internal/common.hpp>
#include <vSMC/helper/sequential.hpp>
#include <tbb/tbb.h>

namespace vSMC {

/// \brief Sampler<T>::init_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T>
class InitializeTBB : public InitializeSeq<T>, InitializeTBBTrait
{
    public :

    InitializeTBB () :
        InitializeSeq<T>() {}

    InitializeTBB (const InitializeTBB<T> &other) :
        InitializeSeq<T>(other) {}

    InitializeTBB<T> &operator= (const InitializeTBB<T> &other)
    {
        InitializeSeq<T>::operator=(other);
        return *this;
    }

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

    typedef typename Particle<T>::size_type size_type;
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
class MoveTBB : public MoveSeq<T>, MoveTBBTrait
{
    public :

    MoveTBB () :
        MoveSeq<T>() {}

    MoveTBB (const MoveTBB<T> &other) :
        MoveSeq<T>(other) {}

    MoveTBB<T> &operator= (const MoveTBB<T> &other)
    {
        MoveSeq<T>::operator=(other);
        return *this;
    }

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

    typedef typename Particle<T>::size_type size_type;
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
class MonitorTBB : public MonitorSeq<T, Dim>, MonitorTBBTrait
{
    public :

    void operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, particle.size()),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);
    }

    private :

    typedef typename Particle<T>::size_type size_type;

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
class PathTBB : public PathSeq<T>, PathTBBTrait
{
    public :

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

    typedef typename Particle<T>::size_type size_type;

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

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_TBB_HPP
