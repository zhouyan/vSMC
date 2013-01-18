#ifndef VSMC_CORE_SAMPLER_HPP
#define VSMC_CORE_SAMPLER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/particle.hpp>

namespace vsmc {

/// \brief SMC Sampler
/// \ingroup Core
template <typename T>
class Sampler
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;
    typedef cxx11::function<unsigned (Particle<T> &, void *)> init_type;
    typedef cxx11::function<unsigned (unsigned, Particle<T> &)> move_type;
    typedef cxx11::function<unsigned (unsigned, Particle<T> &)> mcmc_type;
    typedef std::map<std::string, Monitor<T> > monitor_map_type;

    explicit Sampler (size_type N,
            ResampleScheme scheme = Stratified, double threshold = 0.5) :
        threshold_(threshold), particle_(N), iter_num_(0),
        path_(typename Path<T>::eval_type()), show_(false)
    {
        resample_scheme(scheme);
    }

    explicit Sampler (size_type N,
            const typename Particle<T>::resample_op_type &res_op,
            double threshold = 0.5) :
        threshold_(threshold), particle_(N), iter_num_(0)
    {
        resample_scheme(res_op);
    }

    /// \brief Number of particles
    size_type size () const
    {
        return particle_.size();
    }

    /// \brief Number of iterations (including initialization)
    unsigned iter_size () const
    {
        return static_cast<unsigned>(ess_history_.size());
    }

    /// \brief Set resampling method by a Particle<T>::resample_op_type object
    Sampler<T> &resample_scheme (
            const typename Particle<T>::resample_op_type &res_op)
    {
        particle_.resample_scheme(res_op);

        return *this;
    }

    /// \brief Set resampling method by a built-in ResampleScheme scheme
    /// name
    Sampler<T> &resample_scheme (ResampleScheme scheme)
    {
        particle_.resample_scheme(scheme);

        return *this;
    }

    /// \brief Set resampling method by a scheme name from a collection
    ///
    /// \details
    /// An object of type Resample<ResampleType<EnumType, S>, size_type,
    /// Particle<T>::resample_rng_set_type> will constructed as the resampling
    /// method. This can be a user defined partial specializing of Resample
    /// class template
    ///
    /// For example, resample_scheme<ResampleScheme, Stratified>() is
    /// equivalent to resample_scheme(Stratified)
    template <typename EnumType, EnumType S>
    Sampler<T> &resample_scheme ()
    {
        particle_.template resample_scheme<EnumType, S>();

        return *this;
    }

    /// \brief Set resampling method by the type of resampling object
    ///
    /// \details
    /// An object of type Resample<ResType, size_type,
    /// Particle<T>::rng_set_type>, will constructed as the resampling method.
    /// This can be a user defined partial specializing of Resample class
    /// template
    template <typename ResType>
    Sampler<T> &resample_scheme ()
    {
        particle_.template resample_scheme<ResType>();

        return *this;
    }

    /// \brief Get resampling threshold
    double resample_threshold () const
    {
        return threshold_;
    }

    /// \brief Set resampling threshold
    Sampler<T> &resample_threshold (double threshold)
    {
        threshold_ = threshold;

        return *this;
    }

    /// \brief Get ESS of a given iteration, initialization count as iter 0
    double ess_history (unsigned iter) const
    {
        return ess_history_[iter];
    }

    /// \brief Read ESS history through an output iterator
    template <typename OutputIter>
    OutputIter read_ess_history (OutputIter first) const
    {
        return std::copy(ess_history_.begin(), ess_history_.end(), first);
    }

    /// \brief Get resampling indicator of a given iteration
    bool resampled_history (unsigned iter) const
    {
        return resampled_history_[iter];
    }

    /// \brief Read resampling indicator history through an output iterator
    template <typename OutputIter>
    OutputIter read_resampled_history (OutputIter first) const
    {
        return std::copy(resampled_history_.begin(), resampled_history_.end(),
                first);
    }

    /// \brief Get the number of moves (bosth move and mcmc) of a given
    /// iteration
    unsigned move_num (unsigned iter) const
    {
        return static_cast<unsigned>(accept_history_[iter].size());
    }

    /// \brief Get the accept count of a given move id and the iteration
    ///
    /// \details
    /// The total number of move can be get through move_num(). The first move
    /// performed (either a move or a mcmc) has `id` 0 and so on.
    unsigned accept_history (unsigned id, unsigned iter) const
    {
        return accept_history_[iter][id];
    }

    /// \brief Read and write access to the Particle<T> object
    Particle<T> &particle ()
    {
        return particle_;
    }

    /// \brief Read only access to the Particle<T> object
    const Particle<T> &particle () const
    {
        return particle_;
    }

    /// \brief Set the initialization object of type init_type
    Sampler<T> &init (const init_type &new_init)
    {
        init_ = new_init;

        return *this;
    }

    /// \brief Clear the move queue
    Sampler<T> &move_queue_clear ()
    {
        move_queue_.clear();

        return *this;
    }

    /// \brief Check if move queue is empty
    bool move_queue_empty () const
    {
        return move_queue_.empty();
    }

    /// \brief Check the size of the move queue
    unsigned move_queue_size () const
    {
        return static_cast<unsigned>(move_queue_.size());
    }

    /// \brief Add a new move
    Sampler<T> &move (const move_type &new_move, bool append)
    {
        VSMC_RUNTIME_ASSERT((bool(new_move)),
                "CALL **Sampler::move** WITH AN INVALID MOVE FUNCTOR");
        if (!append)
            move_queue_.clear();
        move_queue_.push_back(new_move);

        return *this;
    }

    /// \brief Add a sequence of new moves
    template <typename InputIter>
    Sampler<T> &move (InputIter first, InputIter last, bool append)
    {
        if (!append)
            move_queue_.clear();
        while (first != last) {
            VSMC_RUNTIME_ASSERT((bool(*first)),
                    "CALL **Sampler::move** WITH AN INVALID MOVE FUNCTOR");
            move_queue_.push_back(*first);
            ++first;
        }

        return *this;
    }

    /// \brief Clear the mcmc queue
    Sampler<T> &mcmc_queue_clear ()
    {
        mcmc_queue_.clear();

        return *this;
    }

    /// \brief Check if mcmc queue is empty
    bool mcmc_queue_empty () const
    {
        return mcmc_queue_.empty();
    }

    /// \brief Check the size of the mcmc queue
    unsigned mcmc_queue_size () const
    {
        return static_cast<unsigned>(mcmc_queue_.size());
    }

    /// \brief Add a new mcmc
    Sampler<T> &mcmc (const mcmc_type &new_mcmc, bool append)
    {
        VSMC_RUNTIME_ASSERT((bool(new_mcmc)),
                "CALL **Sampler::mcmc** WITH AN INVALID MCMC FUNCTOR");
        if (!append)
            mcmc_queue_.clear();
        mcmc_queue_.push_back(new_mcmc);

        return *this;
    }

    /// \brief Add a sequence of new mcmcs
    template <typename InputIter>
    Sampler<T> &mcmc (InputIter first, InputIter last, bool append)
    {
        if (!append)
            mcmc_queue_.clear();
        while (first != last) {
            VSMC_RUNTIME_ASSERT((bool(*first)),
                    "CALL **Sampler::mcmc** WITH AN INVALID MCMC FUNCTOR");
            mcmc_queue_.push_back(*first);
            ++first;
        }

        return *this;
    }

    /// \brief Initialization
    ///
    /// \param param Additional parameters passed to the initialization object
    /// of type init_type
    ///
    /// \note
    /// All histories (ESS, resampled, accept, monitors and path) are clared
    /// before callling the initialization object. Monitors and path's
    /// evaluation objects are untouched.
    Sampler<T> &initialize (void *param = VSMC_NULLPTR)
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
                "CALL **Sampler::initialize** WITH AN INVALID "
                "INITIALIZE FUNCTOR");
        accept_history_.push_back(std::vector<unsigned>(1,
                    init_(particle_, param)));
        do_resampling();
        do_monitoring();
        print_progress();

        return *this;
    }

    /// \brief Iteration
    ///
    /// \details
    /// Moves performed first. Then ESS/N is compared to the threshold and
    /// possible resampling is performed. Then mcmcs are performed. Then
    /// monitors and path are computed
    Sampler<T> &iterate (unsigned num = 1)
    {
        std::vector<unsigned> accept_count;
        for (unsigned i = 0; i != num; ++i) {
            ++iter_num_;
            unsigned ia = 0;
            accept_count.resize(move_queue_.size() + mcmc_queue_.size());

            for (typename std::vector<move_type>::iterator
                    m = move_queue_.begin(); m != move_queue_.end(); ++m) {
                accept_count[ia] = (*m)(iter_num_, particle_);
                ++ia;
            }
            do_resampling();
            for (typename std::vector<mcmc_type>::iterator
                    m = mcmc_queue_.begin(); m != mcmc_queue_.end(); ++m) {
                accept_count[ia] = (*m)(iter_num_, particle_);
                ++ia;
            }
            do_monitoring();
            accept_history_.push_back(accept_count);
            print_progress();
        }

        return *this;
    }

    /// \brief Add a monitor
    ///
    /// \param name The name of the monitor
    /// \param mon The new monitor to be added
    Sampler<T> &monitor (const std::string &name, const Monitor<T> &mon)
    {
        monitor_.insert(std::make_pair(name, mon));

        return *this;
    }

    /// \brief Add a monitor with an evaluation object
    ///
    /// \param name The name of the monitor
    /// \param dim The dimension of the monitor, i.e., the number of variables
    /// \param eval The evaluation object of type Monitor::eval_type
    /// \param method The method of the monitor evaluation
    Sampler<T> &monitor (const std::string &name, unsigned dim,
            const typename Monitor<T>::eval_type &eval,
            MonitorMethod method = ImportanceSampling)
    {
        monitor_.insert(typename monitor_map_type::value_type(
                    name, Monitor<T>(dim, eval, method)));

        return *this;
    }

    /// \brief Read and write access to a named monitor
    Monitor<T> &monitor (const std::string &name)
    {
        typename monitor_map_type::iterator iter = monitor_.find(name);
        VSMC_RUNTIME_ASSERT((iter != monitor_.end()),
                "CALL **Sampler::monitor** WITH AN INVALID MONITOR NAME");

        return iter->second;
    }

    /// \brief Read only access to a named monitor
    const Monitor<T> &monitor (const std::string &name) const
    {
        typename monitor_map_type::const_iterator citer = monitor_.find(name);
        VSMC_RUNTIME_ASSERT((citer != monitor_.end()),
                "CALL **Sampler::monitor** WITH AN INVALID MONITOR NAME");

        return citer->second;
    }

    /// \brief Read and write access to all monitors to the monitor_map_type
    /// object
    monitor_map_type &monitor ()
    {
        return monitor_;
    }

    /// \brief Read only access to all monitors to the the monitor_map_type
    /// object
    const monitor_map_type &monitor () const
    {
        return monitor_;
    }

    /// \brief Erase a named monitor
    bool clear_monitor (const std::string &name)
    {
        return monitor_.erase(name) ==
            static_cast<typename monitor_map_type::size_type>(1);
    }

    /// \brief Erase all monitors
    Sampler<T> &clear_monitor ()
    {
        monitor_.clear();

        return *this;
    }

    /// \brief Read and write access to the Path sampling monitor
    Path<T> &path ()
    {
        return path_;
    }

    /// \brief Read only access to the Path sampling monitor
    const Path<T> &path () const
    {
        return path_;
    }

    /// \brief Set the path sampling evaluation object
    ///
    /// \param eval The evaluation objet of type Path::eval_type
    Sampler<T> &path_sampling (const typename Path<T>::eval_type &eval)
    {
        path_.set_eval(eval);

        return *this;
    }

    /// \brief Path sampling estimate of the logarithm of normalizing constants
    /// ratio
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
    /// \param sampler_id The ID of the sampler
    /// \param sepchar The character used to seperate fields
    /// \param nachar The character used to represent missing values
    ///
    /// \note \c print_path and \c print_monitor are only used to hint the
    /// print process. If there is no record at all, then they won't be printed
    /// even set to \b true instead of being printed all as NA's.
    template<typename OutputStream>
    OutputStream &print (OutputStream &os = std::cout,
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
                        if (m->second.var_name(0).empty()) {
                            os << sepchar << m->first;
                        } else {
                            os << sepchar << m->first << '.'
                                << m->second.var_name(0);
                        }
                    } else {
                        for (unsigned d = 0; d != m->second.dim(); ++d) {
                            if (m->second.var_name(d).empty()) {
                                os << sepchar << m->first << '.' << d + 1;
                            } else {
                                os << sepchar << m->first << '.'
                                    << m->second.var_name(d);
                            }
                        }
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

            os << '\n';
        }

        return os;
    }

    template<typename OutputStream>
    Sampler<T> &print (OutputStream &os = std::cout,
            bool print_header = true,
            bool print_path = true, bool print_monitor = true,
            int sampler_id = 0, char sepchar = ',', char nachar = '\0')
    {
        print(os, print_header, print_path, print_monitor, sampler_id,
                sepchar, nachar);

        return *this;
    }

    /// \brief Set if the sampler shall print dots at each iteration
    Sampler<T> &show_progress (bool show)
    {
        show_ = show;

        return *this;
    }

    private :

    init_type init_;
    std::vector<move_type> move_queue_;
    std::vector<mcmc_type> mcmc_queue_;

    double threshold_;

    Particle<T> particle_;
    unsigned iter_num_;
    std::vector<double> ess_history_;
    std::vector<bool> resampled_history_;
    std::vector<std::vector<unsigned> > accept_history_;

    monitor_map_type monitor_;
    Path<T> path_;

    bool show_;

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
        if (!show_)
            return;

        if (iter_num_ == 0) {
            std::fprintf(stderr, "\n");
            for (int i = 0; i != 78; ++i)
                std::fprintf(stderr, "=");
            std::fprintf(stderr, "\n");
            std::fprintf(stderr, "%6d", iter_num_);
            return;
        }

        if (!(iter_num_ % 50))
            std::fprintf(stderr, "%6d", iter_num_);
        else
            std::fprintf(stderr, ".");

        std::cout.flush();
    }
}; // class Sampler

/// \brief Print the Sampler
/// \ingroup Core
template<typename OutputStream, typename T>
OutputStream &operator<< (OutputStream &os, const Sampler<T> &sampler)
{
    return sampler.print(os, true);
}

} // namespace vsmc

#endif // VSMC_CORE_SAMPLER_HPP
