#ifndef V_SMC_HELPER_PARALLEL_TBB_HPP
#define V_SMC_HELPER_PARALLEL_TBB_HPP

#include <vSMC/internal/common.hpp>
#include <vSMC/helper/sequential.hpp>
#include <tbb/tbb.h>

namespace vSMC {

/// \brief Sampler::init_type subtype
///
/// \tparam T A subtype of StateBase
template <typename T>
class InitializeTBB : public InitializeSeq<T>
{
    public :

    typedef typename InitializeSeq<T>::initialize_state_type
        initialize_state_type;
    typedef typename InitializeSeq<T>::initialize_param_type
        initialize_param_type;
    typedef typename InitializeSeq<T>::pre_processor_type
        pre_processor_type;
    typedef typename InitializeSeq<T>::post_processor_type
        post_processor_type;

    InitializeTBB () {}
    InitializeTBB (const InitializeTBB<T> &other) {}
    InitializeTBB<T> &operator= (const InitializeTBB<T> &other) {return *this;}

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

    typedef V_SMC_INDEX_TYPE size_type;
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

/// \brief Sampler::move_type subtype
///
/// \tparam T A subtype of StateBase
template <typename T>
class MoveTBB : public MoveSeq<T>
{
    public :

    typedef typename MoveSeq<T>::move_state_type     move_state_type;
    typedef typename MoveSeq<T>::pre_processor_type  pre_processor_type;
    typedef typename MoveSeq<T>::post_processor_type post_processor_type;

    MoveTBB () {}
    MoveTBB (const MoveTBB<T> &other) {}
    MoveTBB<T> &operator= (const MoveTBB<T> &other) {return *this;}

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

    typedef V_SMC_INDEX_TYPE size_type;
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

/// \brief Non-direct Monitor::eval_type subtype
///
/// \tparam T A subtype of StateBase
/// \tparam Dim The dimension of the monitor
template <typename T, unsigned Dim>
class MonitorTBB : public MonitorSeq<T, Dim>
{
    public :

    typedef typename MonitorSeq<T>::monitor_state_type monitor_state_type;
    typedef typename MonitorSeq<T>::pre_processor_type pre_processor_type;
    typedef typename MonitorSeq<T>::post_processor_type post_processor_type;

    void operator() (unsigned iter, const Particle<T> &particle,
            double *res)
    {
        this->pre_processor(iter, particle);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, particle.size()),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);
    }

    private :

    typedef V_SMC_INDEX_TYPE size_type;

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

/// \brief Non-direct Path::eval_type subtype
///
/// \tparam T A subtype of StateBase
template <typename T>
class PathTBB : public PathSeq<T>
{
    public :

    typedef typename PathSeq<T>::path_state_type path_state_type;
    typedef typename PathSeq<T>::width_state_type width_state_type;
    typedef typename PathSeq<T>::pre_processor_type pre_processor_type;
    typedef typename PathSeq<T>::post_processor_type post_processor_type;

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

    typedef V_SMC_INDEX_TYPE size_type;

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
