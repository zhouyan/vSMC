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
    typedef typename Particle<T>::size_type size_type;

    /// The type of the particle values
    typedef T value_type;

    /// The type of the Particle set
    typedef Particle<T> particle_type;

    /// The type of the Monitor
    typedef Monitor<T> monitor_type;

    /// The type of the Path sampling monitor
    typedef Path<T> path_type;

    /// The type of initialization functor
    typedef cxx11::function<unsigned (particle_type &, void *)> init_type;

    /// The type of movement functor
    typedef cxx11::function<unsigned (unsigned, particle_type &)> move_type;

    /// An alias to move_type
    typedef move_type mcmc_type;

    /// The type of the movement queue
    typedef std::deque<move_type> move_queue_type;

    /// An alias to move_queue_type
    typedef move_queue_type mcmc_queue_type;

    /// The type of Monitor map
    typedef std::map<std::string, monitor_type> monitor_map_type;

    /// \brief Construct a sampler with a built-in resampling scheme
    ///
    /// \param N The number of particles
    /// \param scheme The built-in resampling scheme. See Resample
    /// \param threshold The threshold of ESS/N for performing resampling. It
    /// shall be a number between [0, 1]. Less than zero means never
    /// resampling, bigger than one means always resampling.
    explicit Sampler (size_type N,
            ResampleScheme scheme = STRATIFIED,
            double threshold = 0.5) :
        threshold_(threshold), particle_(N), iter_num_(0),
        show_progress_(false)
    {
        resample_scheme(scheme);
    }

    /// \brief Construct a sampler with an user defined resampling scheme
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

    /// Size of the Particle set
    size_type size () const
    {
        return particle_.size();
    }

    /// The number of iterations recorded
    ///
    /// \details
    /// This includes the initialization step and started at zero
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
    double ess_history (unsigned iter) const
    {
        return ess_history_[iter];
    }

    /// Read ESS hisotry
    template <typename OutputIter>
    OutputIter read_ess_history (OutputIter first) const
    {
        return std::copy(ess_history_.begin(), ess_history_.end(), first);
    }

    /// Resampling history
    bool resampled_history (unsigned iter) const
    {
        return resampled_history_[iter];
    }

    /// Read ESS hisotry
    template <typename OutputIter>
    OutputIter read_resampled_history (OutputIter first) const
    {
        return std::copy(resampled_history_.begin(), resampled_history_.end(),
                first);
    }

    /// Number of moves
    unsigned move_num (unsigned iter) const
    {
        return accept_history_[iter].size();
    }

    /// Accept count history
    unsigned accept_history (unsigned iter, unsigned move_num) const
    {
        return accept_history_[iter][move_num];
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

    /// Set a new initialization functor
    void init (const init_type &new_init)
    {
        init_ = new_init;
    }

    /// Clear the movement queue and set a new movement functor
    void move (const move_type &new_move)
    {
        move_queue_.clear();
        move_queue_.push_back(new_move);
    }

    /// Read and write access to the movement queue
    move_queue_type &move_queue ()
    {
        return move_queue_;
    }

    /// Read only access to the movement queue
    const move_queue_type &move_queue () const
    {
        return move_queue_;
    }

    /// Clear the MCMC movement queue and set a new MCMC movement functor
    void mcmc (const mcmc_type &new_mcmc)
    {
        mcmc_queue_.clear();
        mcmc_queue_.push_back(new_mcmc);
    }

    /// Read and write access to the MCMC movement queue
    move_queue_type &mcmc_queue ()
    {
        return mcmc_queue_;
    }

    /// Read only access to the MCMC movement queue
    const move_queue_type &mcmc_queue () const
    {
        return mcmc_queue_;
    }

    /// \brief Initialize the particle set
    ///
    /// \param param Additional parameters passed to the initialization
    /// functor
    ///
    /// \details
    /// Before calling the initialization functor, set by Sampler::init, all
    /// histories (ESS, resampling, accet counts) are cleared. Monitor::clear
    /// are called upon each monitor as well.
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
        accept_history_.push_back(std::vector<unsigned>(1,
                    init_(particle_, param)));
        do_resampling();
        do_monitoring();
        print_progress();
    }

    /// Perform iteration for a given number times
    ///
    /// \details
    /// Movements set through Sampler::move and Sampler::move_queue are
    /// performed first. Then threshold of ESS/N is checked and possible
    /// resampling is performed. After that, movements set through
    /// Sampler::mcmc and Sampler::mcmc_queue are performed.
    void iterate (unsigned num = 1)
    {
        std::vector<unsigned> accept_count;
        for (unsigned i = 0; i != num; ++i) {
            ++iter_num_;
            unsigned ia = 0;
            accept_count.resize(move_queue_.size() + mcmc_queue_.size());

            for (typename move_queue_type::iterator
                    m = move_queue_.begin(); m != move_queue_.end(); ++m) {
                VSMC_RUNTIME_ASSERT((bool(*m)),
                        ("CALL **Sampler::iterate** WITH AN INVALID "
                         "MOVE FUNCTOR"));
                accept_count[ia] = (*m)(iter_num_, particle_);
                ++ia;
            }
            do_resampling();
            for (typename mcmc_queue_type::iterator
                    m = mcmc_queue_.begin(); m != mcmc_queue_.end(); ++m) {
                VSMC_RUNTIME_ASSERT((bool(*m)),
                        ("CALL **Sampler::iterate** WITH AN INVALID "
                         "MCMC FUNCTOR"));
                accept_count[ia] = (*m)(iter_num_, particle_);
                ++ia;
            }
            do_monitoring();
            accept_history_.push_back(accept_count);
            print_progress();
        }
    }

    /// \brief Add a monitor
    ///
    /// \param name The name of the monitor
    /// \param mon The new monitor to be added
    void monitor (const std::string &name, const monitor_type &mon)
    {
        monitor_.insert(std::make_pair(name, mon));
    }

    /// \brief Add a monitor with a evaluation functor
    ///
    /// \param name The name of the monitor
    /// \param dim The dimension of the monitor, i.e., the number of variables
    /// \param eval The functor used to evaluate the results
    ///
    /// \sa Monitor::eval_type
    void monitor (const std::string &name, unsigned dim,
            const typename monitor_type::eval_type &eval)
    {
        monitor_.insert(std::make_pair(name, monitor_type(dim, eval)));
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
    }

    /// Erase all monitors
    void clear_monitor ()
    {
        monitor_.clear();
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

    /// \brief Print the history of the Sampler
    ///
    /// \param os The ostream to which the contents are printed
    /// \param print_header Print header if \b true
    /// \param print_path Print path sampling if \b true
    /// \param print_monitor Print monitors if \b true
    /// \param sampler_id The ID to be printed if \c print_id is \b true
    /// \param sepchar The character used to seperate fields
    /// \param nachar The character used to represent missing values
    ///
    /// \note \c print_path and \c print_monitor are only used to hint the
    /// print process. If there is no record at all, then they won't be printed
    /// even set to \b true.
    template<typename OutputStream>
    void print (OutputStream &os = std::cout,
            bool print_header = true,
            bool print_path = true, bool print_monitor = true,
            int sampler_id = 0, char sepchar = ',', char nachar = '\0') const
    {
        // Accept count
        std::vector<double> acc;
        unsigned accd = 0;
        for (unsigned iter = 0; iter != iter_size(); ++iter) {
            accd = std::max(
                    accd, static_cast<unsigned>(accept_history_[iter].size()));
        }
        bool print_accept = accd > 0 && iter_size() > 0;

        // Path sampling
        print_path = print_path && path_.iter_size() > 0 && iter_size() > 0;
        std::vector<long> path_mask;
        if (print_path) {
            path_mask.resize(iter_size(), -1);
            for (unsigned d = 0; d != path_.iter_size(); ++d)
                path_mask[path_.index(d)] = d;
        }

        // Monitors
        unsigned mond = 0;
        unsigned mi = 0;
        for (typename monitor_map_type::const_iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            mond += m->second.dim();
            mi = std::max(mi, static_cast<unsigned>(m->second.iter_size()));
        }
        print_monitor = print_monitor && mond > 0 && mi > 0 && iter_size() > 0;
        std::vector<std::vector<long> > monitor_mask;
        if (print_monitor) {
            monitor_mask.resize(monitor_.size());
            unsigned mm = 0;
            for (typename monitor_map_type::const_iterator
                    m = monitor_.begin(); m != monitor_.end(); ++m) {
                monitor_mask[mm].resize(iter_size(), -1);
                for (unsigned d = 0; d != m->second.iter_size(); ++d)
                    monitor_mask[mm][m->second.index(d)] = d;
                ++mm;
            }
        }

        // Print header
        if (print_header) {
            os << "Sampler.ID";
            os << sepchar << "Iter";
            os << sepchar << "ESS";
            os << sepchar << "ResSam";
            if (print_accept) {
                if (accd == 1) {
                    os << sepchar << "Accept";
                } else {
                    for (unsigned d = 0; d != accd; ++d)
                        os << sepchar << "Accept." << d + 1;
                }
            }
            if (print_path) {
                os << sepchar << "Path.Integrand";
                os << sepchar << "Path.Width";
                os << sepchar << "Path.Grid";
            }
            if (print_monitor) {
                for (typename monitor_map_type::const_iterator
                        m = monitor_.begin(); m != monitor_.end(); ++m) {
                    if (m->second.dim() == 1) {
                        os << sepchar << m->first;
                    } else {
                        for (unsigned d = 0; d != m->second.dim(); ++d)
                            os << sepchar << m->first << '.' << d + 1;
                    }
                }
            }
            if (iter_size() > 0)
                os << '\n';
        }

        // Print data
        for (unsigned iter = 0; iter != iter_size(); ++iter) {
            os << sampler_id;
            os << sepchar << iter;
            os << sepchar << ess_history_[iter] / size();
            os << sepchar << resampled_history_[iter];

            if (print_accept) {
                for (unsigned c = 0; c != accept_history_[iter].size(); ++c)
                    os << sepchar <<
                        accept_history_[iter][c] / static_cast<double>(size());
                unsigned diff = static_cast<unsigned>(
                        accd - accept_history_[iter].size());
                for (unsigned c = 0; c != diff; ++c)
                    os << sepchar << 0;
            }

            if (print_path) {
                long pr = path_mask[iter];
                if (pr >= 0) {
                    os << sepchar << path_.integrand(pr);
                    os << sepchar << path_.width(pr);
                    os << sepchar << path_.grid(pr);
                } else {
                    os << sepchar << nachar;
                    os << sepchar << nachar;
                    os << sepchar << nachar;
                }
            }

            if (print_monitor) {
                unsigned mm = 0;
                for (typename monitor_map_type::const_iterator
                        m = monitor_.begin(); m != monitor_.end(); ++m) {
                    long mr = monitor_mask[mm][iter];
                    if (mr >= 0) {
                        for (unsigned d = 0; d != m->second.dim(); ++d)
                            os << sepchar << m->second.record(d, mr);
                    } else {
                        for (unsigned d = 0; d != m->second.dim(); ++d)
                            os << sepchar << nachar;
                    }
                    ++mm;
                }
            }

            if (iter + 1 < iter_size())
                os << '\n';
        }
    }

    void show_progress (bool show)
    {
        show_progress_ = show;
    }

    private :

    init_type init_;
    move_queue_type move_queue_;
    mcmc_queue_type mcmc_queue_;

    double threshold_;

    particle_type particle_;
    unsigned iter_num_;
    std::vector<double> ess_history_;
    std::vector<bool> resampled_history_;
    std::vector<std::vector<unsigned> > accept_history_;

    monitor_map_type monitor_;
    path_type path_;

    bool show_progress_;

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

    void print_progress () const
    {
        if (!show_progress_)
            return;

        if (iter_num_ == 0) {
            std::cout << std::endl;
            for (int i = 0; i != 60; ++i)
                std::cout << '=';
            std::cout << std::endl;
            std::cout << std::setw(6) << iter_num_;
            return;
        }

        if (!(iter_num_ % 50))
            std::cout << std::endl << std::setw(6) << iter_num_;
        else
            std::cout << ".";

        std::cout.flush();
    }
}; // class Sampler

/// \brief Print the Sampler
/// \ingroup Core
///
/// \param os The ostream to which the contents are printed
/// \param sampler The Sampler to be printed
///
/// \note This is the same as <tt>sampler.print(os)</tt>
template<typename OutputStream, typename T>
OutputStream &operator<< (OutputStream &os, const vsmc::Sampler<T> &sampler)
{
    sampler.print(os, true);
    return os;
}

} // namespace vsmc

#endif // VSMC_CORE_SAMPLER_HPP
