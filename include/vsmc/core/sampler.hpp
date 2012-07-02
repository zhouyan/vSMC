#ifndef VSMC_CORE_SAMPLER_HPP
#define VSMC_CORE_SAMPLER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/particle.hpp>

namespace vsmc {

/// \brief SMC Sampler
/// \ingroup Core
///
/// \tparam T Particle<T>::value_type
template <typename T>
class Sampler
{
    public :

    /// The type of the number of particles
    typedef typename SizeTypeTrait<T>::type size_type;

    /// The type of the particle values
    typedef T value_type;

    /// The type of the particle set
    typedef Particle<T> particle_type;

    /// The type of the monitor
    typedef Monitor<T> monitor_type;

    /// The type of the path
    typedef Path<T> path_type;

    /// The type of Initialization functor
    typedef internal::function<unsigned (particle_type &, void *)> init_type;

    /// The type of Move functor
    typedef internal::function<unsigned (unsigned, particle_type &)> move_type;

    /// The type of MCMC functor
    typedef internal::function<unsigned (unsigned, particle_type &)> mcmc_type;

    /// The type of the Move queue
    typedef std::deque<move_type> move_queue_type;

    /// The type of the MCMC queue
    typedef std::deque<mcmc_type> mcmc_queue_type;

    /// The type of ESS history vector
    typedef std::vector<double> ess_history_type;

    /// The type of resampling history vector
    typedef std::vector<bool> resampled_history_type;

    /// The type of accept count history vector
    typedef std::vector<std::deque<unsigned> > accept_history_type;

    /// The type of Monitor map
    typedef std::map<std::string, monitor_type> monitor_map_type;

    /// \brief Construct a sampler with a given number of particles
    ///
    /// \param N The number of particles
    /// \param scheme The built-in resampling scheme. See Resample
    /// \param threshold The threshold of ESS/N for performing resampling. It
    /// shall be a number between [0, 1]. Less than zero means never
    /// resampling, bigger than one means always resampling.
    explicit Sampler (size_type N,
            ResampleScheme scheme = STRATIFIED,
            double threshold = 0.5) :
        threshold_(threshold), particle_(N), iter_num_(0)
    {
        resample_scheme(scheme);
    }

    /// \brief Construct a sampler with a given number of particles
    ///
    /// \param N The number of particles
    /// \param res_op A resampling operation functor
    /// \param threshold The threshold of ESS/N for performing resampling.
    explicit Sampler (size_type N,
            const typename particle_type::resample_op_type &res_op,
            double threshold = 0.5) :
        threshold_(threshold), particle_(N), iter_num_(0)
    {
        resample_scheme(res_op);
    }

    /// Size of the particle set
    size_type size () const
    {
        return particle_.size();
    }

    /// The number of iterations recorded (including initialization)
    unsigned iter_size () const
    {
        return static_cast<unsigned>(ess_history_.size());
    }

    /// See Particle::resample_scheme
    void resample_scheme (
            const typename particle_type::resample_op_type &res_op)
    {
        particle_.resample_scheme(res_op);
    }

    /// See Particle::resample_scheme
    void resample_scheme (ResampleScheme scheme)
    {
        particle_.resample_scheme(scheme);
    }

    /// See Particle::resample_scheme
    template <typename EnumType, EnumType S>
    void resample_scheme ()
    {
        particle_.template resample_scheme<EnumType, S>();
    }

    /// See Particle::resample_scheme
    template <typename ResType>
    void resample_scheme ()
    {
        particle_.template resample_scheme<ResType>();
    }

    /// The current threshold
    double resample_threshold () const
    {
        return threshold_;
    }

    /// Set new resampling threshold
    void resample_threshold (double threshold)
    {
        threshold_ = threshold;
    }

    /// ESS history
    const ess_history_type &ess_history () const
    {
        return ess_history_;
    }

    /// Resampling history
    const resampled_history_type &resampled_history () const
    {
        return resampled_history_;
    }

    /// Accept count history
    const accept_history_type &accept_history () const
    {
        return accept_history_;
    }

    /// Read and write access to the particle set
    particle_type &particle ()
    {
        return particle_;
    }

    /// Read only access to the particle set
    const particle_type &particle () const
    {
        return particle_;
    }

    /// Set a new Initialization
    void init (const init_type &new_init)
    {
        init_ = new_init;
    }

    /// Clear the Move queue and set a single new Move
    void move (const move_type &new_move)
    {
        move_queue_.clear();
        move_queue_.push_back(new_move);
    }

    /// Read and write access to the Move queue
    move_queue_type &move_queue ()
    {
        return move_queue_;
    }

    /// Read only access to the Move queue
    const move_queue_type &move_queue () const
    {
        return move_queue_;
    }

    /// Clear the MCMC queue and set a single new MCMC
    void mcmc (const mcmc_type &new_mcmc)
    {
        mcmc_queue_.clear();
        mcmc_queue_.push_back(new_mcmc);
    }

    /// Read and write access to the MCMC queue
    mcmc_queue_type &mcmc_queue ()
    {
        return mcmc_queue_;
    }

    /// Read only access to the MCMC queue
    const mcmc_queue_type &mcmc_queue () const
    {
        return mcmc_queue_;
    }

    /// \brief Initialize the particle set
    ///
    /// \param param Additional parameters passed to the initialization
    /// functor
    void initialize (void *param = VSMC_NULLPTR)
    {
        ess_history_.clear();
        resampled_history_.clear();
        accept_history_.clear();
        path_.clear();

        for (typename monitor_map_type::iterator
                m = monitor_.begin(); m != monitor_.end(); ++m)
            m->second.clear();

        iter_num_ = 0;
        VSMC_RUNTIME_ASSERT((bool(init_)),
                ("CALL **Sampler::initialize** WITH AN INVALID "
                 "INITIALIZE FUNCTOR"));
        accept_history_.push_back(
                std::deque<unsigned>(1, init_(particle_, param)));
        do_resampling();
        do_monitoring();
        particle_.reset_zconst();
    }

    /// Perform iteration for a given number times
    void iterate (unsigned num = 1)
    {
        for (unsigned i = 0; i != num; ++i) {
            ++iter_num_;
            unsigned ia = 0;
            std::deque<unsigned> acc(move_queue_.size() + mcmc_queue_.size());

            for (typename move_queue_type::iterator
                    m = move_queue_.begin(); m != move_queue_.end(); ++m) {
                VSMC_RUNTIME_ASSERT((bool(*m)),
                        ("CALL **Sampler::iterate** WITH AN INVALID "
                         "MOVE FUNCTOR"));
                acc[ia] = (*m)(iter_num_, particle_);
                ++ia;
            }

            do_resampling();

            for (typename mcmc_queue_type::iterator
                    m = mcmc_queue_.begin(); m != mcmc_queue_.end(); ++m) {
                VSMC_RUNTIME_ASSERT((bool(*m)),
                        ("CALL **Sampler::iterate** WITH AN INVALID "
                         "MCMC FUNCTOR"));
                acc[ia] = (*m)(iter_num_, particle_);
                ++ia;
            }

            accept_history_.push_back(acc);

            do_monitoring();
        }
    }

    /// \brief Add a monitor with a evaluation functor
    ///
    /// \param name The name of the monitor
    /// \param dim The dimension of the monitor, i.e., the number of variables
    /// \param eval The functor used to evaluate the results
    ///
    /// \sa Monitor<T>
    void monitor (const std::string &name, unsigned dim,
            const typename monitor_type::eval_type &eval)
    {
        monitor_.insert(std::make_pair(name, monitor_type(dim, eval)));
        monitor_name_.insert(name);
    }

    /// Read only access to a named monitor through an iterator
    typename monitor_map_type::const_iterator monitor (
            const std::string &name) const
    {
        return monitor_.find(name);
    }

    /// Read and write access to a named monitor through iterator
    typename monitor_map_type::iterator monitor (const std::string &name)
    {
        return monitor_.find(name);
    }

    /// Read only access to all monitors
    const monitor_map_type &monitor () const
    {
        return monitor_;
    }

    /// Read and write access to all monitors
    monitor_map_type &monitor ()
    {
        return monitor_;
    }

    /// Erase a named monitor
    void clear_monitor (const std::string &name)
    {
        monitor_.erase(name);
        monitor_name_.erase(name);
    }

    /// Erase all monitors
    void clear_monitor ()
    {
        monitor_.clear();
        monitor_name_.clear();
    }

    /// Read only access to the Path sampling monitor
    const path_type &path () const
    {
        return path_;
    }

    /// Read and write access to the Path sampling monitor
    path_type &path ()
    {
        return path_;
    }

    /// \brief Set the path sampling evaluation functor
    ///
    /// \param eval The functor used to compute the integrands or results
    ///
    /// \sa Path::eval_type
    void path_sampling (const typename path_type::eval_type &eval)
    {
        path_.set_eval(eval);
    }

    /// Path sampling estimate of the logarithm of normalizing constants ratio
    double path_sampling () const
    {
        return path_.zconst();
    }

    /// SMC estimate of the logarithm of normalizing constants ratio
    double zconst () const
    {
        return particle_.zconst();
    }

    /// \brief Print the history of the Sampler
    ///
    /// \param os The ostream to which the contents are printed
    /// \param print_header Print header if \b true
    /// \param print_path Print path sampling if \b true
    /// \param print_monitor Print monitors if \b true
    ///
    /// \note \c print_path and \c print_monitor are only used to hint the
    /// print process. If there is no record at all, then they won't be printed
    /// even set to \b true.
    template<typename CharT, typename Traits>
    void print (std::basic_ostream<CharT, Traits> &os = std::cout,
            bool print_header = true,
            bool print_path = true, bool print_monitor = true) const
    {
        const char sep = '\t';

        // Accept count
        Eigen::MatrixXd acc;
        unsigned accd = 0;
        for (accept_history_type::const_iterator
                a = accept_history_.begin(); a != accept_history_.end(); ++a) {
            if (a->size() > accd)
                accd = static_cast<unsigned>(a->size());
        }
        bool print_accept = accd > 0 && iter_size() > 0;
        if (print_accept) {
            acc.resize(iter_size(), accd);
            acc.setConstant(0);
            double accdnorm = static_cast<double>(size());
            for (unsigned r = 0; r != iter_size(); ++r)
                for (unsigned c = 0; c != accept_history_[r].size(); ++c)
                    acc(r, c) = accept_history_[r][c] / accdnorm;
        }

        // Path sampling
        Eigen::MatrixXd pa;
        Eigen::MatrixXi pmask;
        print_path = print_path && path_.iter_size() > 0 && iter_size() > 0;
        if (print_path) {
            pa.resize(iter_size(), 3);
            pmask.resize(iter_size(), 1);
            pmask.setConstant(0);
            for (unsigned d = 0; d != path_.iter_size(); ++d) {
                unsigned pr = path_.index()[d];
                pa(pr, 0) = path_.integrand()[d];
                pa(pr, 1) = path_.width()[d];
                pa(pr, 2) = path_.grid()[d];
                pmask(pr) = 1;
            }
        }

        // Monitors
        Eigen::MatrixXd mon;
        Eigen::MatrixXi mmask;
        unsigned mond = 0;
        for (typename monitor_map_type::const_iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            mond += m->second.dim();
        }
        print_monitor = print_monitor && mond > 0 && iter_size() > 0;
        if (print_monitor) {
            mon.resize(iter_size(), mond);
            mmask.resize(iter_size(), monitor_.size());
            mmask.setConstant(0);
            unsigned mc = 0;
            unsigned mn = 0;
            for (typename monitor_map_type::const_iterator
                    m = monitor_.begin(); m != monitor_.end(); ++m) {
                unsigned md = m->second.dim();
                for (unsigned d = 0; d != m->second.iter_size(); ++d) {
                    unsigned mr = m->second.index()[d];
                    for (unsigned c = 0; c != md; ++c)
                        mon(mr, c + mc) = m->second.record(c)[d];
                    mmask(mr, mn) = 1;
                }
                mc += md;
                ++mn;
            }
        }

        // Print header
        if (print_header) {
            os << "Iter" << sep << "ESS" << sep << "ResSam" << sep;
            if (print_accept) {
                if (accd == 1) {
                    os << "Accept" << sep;
                } else {
                    for (unsigned d = 0; d != accd; ++d)
                        os << "Accept." << d + 1 << sep;
                }
            }
            if (print_path) {
                os
                    << "Path.Integrand" << sep
                    << "Path.Width" << sep
                    << "Path.Grid" << sep << "";
            }
            if (print_monitor) {
                for (typename monitor_map_type::const_iterator
                        m = monitor_.begin(); m != monitor_.end(); ++m) {
                    if (m->second.dim() == 1) {
                        os << m->first << sep;
                    } else {
                        for (unsigned d = 0; d != m->second.dim(); ++d)
                            os << m->first << '.' << d + 1 << sep;
                    }
                }
            }
            if (iter_size() > 0)
                os << '\n';
        }

        // Print data
        for (unsigned iter = 0; iter != iter_size(); ++iter) {
            os << iter << sep;
            os << ess_history_[iter] / size() << sep;
            os << resampled_history_[iter] << sep;
            if (print_accept)
                os << acc.row(iter) << sep;
            if (print_path) {
                if (pmask(iter))
                    os << pa.row(iter) << sep;
                else
                    os << '.' << sep << '.' << sep << '.' << sep;
            }
            if (print_monitor) {
                unsigned mc = 0;
                unsigned mn = 0;
                for (typename monitor_map_type::const_iterator
                        m = monitor_.begin(); m != monitor_.end(); ++m) {
                    unsigned md = m->second.dim();
                    if (mmask(iter, mn)) {
                        os << mon.block(iter, mc, 1, md) << sep;
                    } else {
                        for (unsigned m = 0; m != md; ++m)
                            os << '.' << sep;
                    }
                    mc += md;
                    ++mn;
                }
            }
            if (iter + 1 < iter_size())
                os << '\n';
        }
    }

    private :

    init_type init_;
    move_queue_type move_queue_;
    mcmc_queue_type mcmc_queue_;

    double threshold_;

    particle_type particle_;
    unsigned iter_num_;
    ess_history_type ess_history_;
    resampled_history_type resampled_history_;
    accept_history_type accept_history_;

    monitor_map_type monitor_;
    std::set<std::string> monitor_name_;
    path_type path_;

    void do_resampling ()
    {
        particle_.resample(threshold_);
        ess_history_.push_back(particle_.ess());
        resampled_history_.push_back(particle_.resampled());
    }

    void do_monitoring ()
    {
        if (bool(path_))
            path_.eval(iter_num_, particle_);

        for (typename monitor_map_type::iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            if (bool(m->second))
                m->second.eval(iter_num_, particle_);
        }
    }
}; // class Sampler

/// \brief Print the Sampler
/// \ingroup Core
///
/// \param os The ostream to which the contents are printed
/// \param sampler The Sampler to be printed
///
/// \note This is the same as <tt>sampler.print(os)</tt>
template<typename CharT, typename Traits, typename T>
std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const vsmc::Sampler<T> &sampler)
{
    sampler.print(os, true);
    return os;
}

} // namespace vsmc

#endif // VSMC_CORE_SAMPLER_HPP
