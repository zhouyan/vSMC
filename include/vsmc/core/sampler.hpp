#ifndef VSMC_CORE_SAMPLER_HPP
#define VSMC_CORE_SAMPLER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/path.hpp>

namespace vsmc {

/// \brief SMC Sampler
/// \ingroup Core
template <typename T>
class Sampler
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;
    typedef cxx11::function<std::size_t (Particle<T> &, void *)> init_type;
    typedef cxx11::function<std::size_t (std::size_t, Particle<T> &)>
        move_type;
    typedef cxx11::function<std::size_t (std::size_t, Particle<T> &)>
        mcmc_type;
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

    /// \brief Reserve space for a specified number of iterations
    void reserve (std::size_t num)
    {
        ess_history_.reserve(num);
        resampled_history_.reserve(num);
        for (std::size_t i = 0; i != accept_history_.size(); ++i)
            accept_history_[i].reserve(num);
        if (bool(path_))
            path_.reserve(num);
        for (typename monitor_map_type::iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            if (bool(m->second))
                m->second.reserve(num);
        }
    }

    /// \brief Number of iterations (including initialization)
    std::size_t iter_size () const
    {
        return ess_history_.size();
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
    double ess_history (std::size_t iter) const
    {
        return ess_history_[iter];
    }

    /// \brief Read ESS history through an output iterator
    template <typename OutputIter>
    OutputIter read_ess_history (OutputIter first) const
    {
        for (std::size_t i = 0; i != ess_history_.size(); ++i, ++first)
            *first = ess_history_[i];

        return first;
    }

    /// \brief Get resampling indicator of a given iteration
    bool resampled_history (std::size_t iter) const
    {
        return resampled_history_[iter];
    }

    /// \brief Read resampling indicator history through an output iterator
    template <typename OutputIter>
    OutputIter read_resampled_history (OutputIter first) const
    {
        for (std::size_t i = 0; i != resampled_history_.size(); ++i, ++first)
            *first = resampled_history_[i];

        return first;
    }

    /// \brief Get the number of moves (both move and mcmc) of a given
    /// iteration
    std::size_t move_num (std::size_t iter) const
    {
        return accept_history_[iter].size();
    }

    /// \brief Get the accept count of a given move id and the iteration
    ///
    /// \details
    /// The total number of move can be get through move_num(). The first move
    /// performed (either a move or a mcmc) has `id` 0 and so on.
    std::size_t accept_history (std::size_t id, std::size_t iter) const
    {
        return accept_history_[id][iter];
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
        VSMC_RUNTIME_ASSERT_FUNCTOR(new_init, Sampler::init, Initialize);

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
    std::size_t move_queue_size () const
    {
        return move_queue_.size();
    }

    /// \brief Add a new move
    Sampler<T> &move (const move_type &new_move, bool append)
    {
        VSMC_RUNTIME_ASSERT_FUNCTOR(new_move, Sampler::move, MOVE);

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
            VSMC_RUNTIME_ASSERT_FUNCTOR(*first, Sampler::move, MOVE);
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
    std::size_t mcmc_queue_size () const
    {
        return mcmc_queue_.size();
    }

    /// \brief Add a new mcmc
    Sampler<T> &mcmc (const mcmc_type &new_mcmc, bool append)
    {
        VSMC_RUNTIME_ASSERT_FUNCTOR(new_mcmc, Sampler::mcmc, MCMC);

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
            VSMC_RUNTIME_ASSERT_FUNCTOR(*first, Sampler::mcmc, MCMC);
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
    /// All histories (ESS, resampled, accept, Monitor and Path) are clared
    /// before callling the initialization object. Monitors and Path's
    /// evaluation objects are untouched.
    Sampler<T> &initialize (void *param = VSMC_NULLPTR)
    {
        VSMC_RUNTIME_ASSERT_FUNCTOR(init_, Sampler::initialize, Initialize);

        ess_history_.clear();
        resampled_history_.clear();
        accept_history_.clear();
        path_.clear();
        for (typename monitor_map_type::iterator
                m = monitor_.begin(); m != monitor_.end(); ++m)
            m->second.clear();

        iter_num_ = 0;
        accept_history_.push_back(std::vector<std::size_t>(1,
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
    /// monitors and Path are computed
    Sampler<T> &iterate (std::size_t num = 1)
    {
        if (num > 1)
            reserve(iter_size() + num);

        if (accept_history_.size() < move_queue_.size() + mcmc_queue_.size()) {
            std::size_t diff = move_queue_.size() + mcmc_queue_.size() -
                accept_history_.size();
            for (std::size_t i = 0; i != diff; ++i) {
                accept_history_.push_back(
                        std::vector<std::size_t>(iter_size(), 0));
            }
        }

        for (std::size_t i = 0; i != num; ++i) {
            ++iter_num_;
            std::size_t ia = 0;
            for (typename std::vector<move_type>::iterator
                    m = move_queue_.begin(); m != move_queue_.end(); ++m) {
                accept_history_[ia].push_back((*m)(iter_num_, particle_));
                ++ia;
            }
            do_resampling();
            for (typename std::vector<mcmc_type>::iterator
                    m = mcmc_queue_.begin(); m != mcmc_queue_.end(); ++m) {
                accept_history_[ia].push_back((*m)(iter_num_, particle_));
                ++ia;
            }
            for (; ia != accept_history_.size(); ++ia)
                accept_history_[ia].push_back(0);
            do_monitoring();
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
    Sampler<T> &monitor (const std::string &name, std::size_t dim,
            const typename Monitor<T>::eval_type &eval)
    {
        monitor_.insert(typename monitor_map_type::value_type(
                    name, Monitor<T>(dim, eval)));

        return *this;
    }

    /// \brief Read and write access to a named monitor
    Monitor<T> &monitor (const std::string &name)
    {
        typename monitor_map_type::iterator iter = monitor_.find(name);

        VSMC_RUNTIME_ASSERT_MONITOR_NAME(iter, monitor_, Sampler::monitor);

        return iter->second;
    }

    /// \brief Read only access to a named monitor
    const Monitor<T> &monitor (const std::string &name) const
    {
        typename monitor_map_type::const_iterator citer = monitor_.find(name);

        VSMC_RUNTIME_ASSERT_MONITOR_NAME(citer, monitor_, Sampler::monitor);

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

    /// \brief Set the Path sampling evaluation object
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

    /// \brief Set if the sampler shall print dots at each iteration
    Sampler<T> &show_progress (bool show)
    {
        show_ = show;

        return *this;
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
            bool print_path = true,
            bool print_monitor = true,
            std::size_t sampler_id = 0,
            char sepchar = ',', char nachar = '\0') const
    {
        // Accept count
        bool print_accept = accept_history_.size() > 0 && iter_size() > 0;

        // Path sampling
        print_path = print_path && path_.iter_size() > 0 && iter_size() > 0;

        // Monitors
        std::size_t mond = 0;
        std::size_t miter = 0;
        if (print_monitor) {
            for (typename monitor_map_type::const_iterator
                    m = monitor_.begin(); m != monitor_.end(); ++m) {
                mond += m->second.dim();
                miter += m->second.iter_size();
            }
        }
        print_monitor = mond > 0 && miter > 0 && iter_size() > 0;

        // Print header
        if (print_header) {
            std::vector<std::string> header(get_print_header(
                        print_accept, print_path, print_monitor));
            os << header[0];
            for (std::size_t i = 1; i != header.size(); ++i)
                os << sepchar << header[i];
            if (iter_size() > 0)
                os << '\n';
        }

        // Print data
        std::pair<std::vector<double>, std::vector<bool> > data(
                get_print_data(print_accept, print_path, print_monitor));
        std::size_t data_dim = data.first.size() / iter_size();
        std::size_t data_offset = 0;
        for (std::size_t iter = 0; iter != iter_size(); ++iter) {
            os << sampler_id;
            os << sepchar << iter;
            os << sepchar << resampled_history_[iter];
            for (std::size_t i = 0; i != data_dim; ++i) {
                if (data.second[data_offset])
                    os << sepchar << data.first[data_offset];
                else
                    os << sepchar << nachar;
                ++data_offset;
            }
            os << '\n';
        }

        return os;
    }

    private :

    init_type init_;
    std::vector<move_type> move_queue_;
    std::vector<mcmc_type> mcmc_queue_;

    double threshold_;

    Particle<T> particle_;
    std::size_t iter_num_;
    std::vector<double> ess_history_;
    std::vector<bool> resampled_history_;
    std::vector<std::vector<std::size_t> > accept_history_;

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
            std::fprintf(stderr, "%6u", static_cast<unsigned>(iter_num_));
            return;
        }

        if (!(iter_num_ % 50))
            std::fprintf(stderr, "%6u", static_cast<unsigned>(iter_num_));
        else
            std::fprintf(stderr, ".");

        std::cout.flush();
    }

    std::vector<std::string> get_print_header (
            bool print_accept, bool print_path, bool print_monitor) const
    {
        std::size_t header_length = 4;
        if (print_accept)
            header_length += accept_history_.size();
        if (print_path)
            header_length += 2;
        if (print_monitor) {
            for (typename monitor_map_type::const_iterator
                    m = monitor_.begin(); m != monitor_.end(); ++m) {
                header_length += m->second.dim();
            }
        }
        std::vector<std::string> header;
        header.reserve(header_length);

        header.push_back(std::string("Sampler.ID"));
        header.push_back(std::string("Iter.Num"));
        header.push_back(std::string("Resampled"));
        header.push_back(std::string("ESS"));
        if (print_accept) {
            char name[32];
            unsigned accd = static_cast<unsigned>(accept_history_.size());
            for (unsigned i = 0; i != accd; ++i) {
                std::sprintf(name, "Accept.%u", i);
                header.push_back(std::string(name));
            }
        }
        if (print_path) {
            header.push_back(std::string("Path.Integrand"));
            header.push_back(std::string("Path.Grid"));
        }
        if (print_monitor) {
            for (typename monitor_map_type::const_iterator
                    m = monitor_.begin(); m != monitor_.end(); ++m) {
                char *name = new char[m->first.size() + 15];
                unsigned mond = static_cast<unsigned>(m->second.dim());
                for (unsigned i = 0; i != mond; ++i) {
                    std::sprintf(name, "%s.%u", m->first.c_str(), i);
                    header.push_back(std::string(name));
                }
                delete [] name;
            }
        }

        return header;
    }

    std::pair<std::vector<double>, std::vector<bool> > get_print_data (
            bool print_accept, bool print_path, bool print_monitor) const
    {
        std::size_t dim = 1;
        if (print_accept)
            dim += accept_history_.size();
        if (print_path)
            dim += 2;
        if (print_monitor) {
            for (typename monitor_map_type::const_iterator
                    m = monitor_.begin(); m != monitor_.end(); ++m) {
                dim += m->second.dim();
            }
        }

        std::vector<double> data;
        std::vector<bool> mask;
        data.reserve(dim * iter_size());
        mask.reserve(dim * iter_size());

        std::size_t piter = 0;
        std::vector<std::size_t> miter(monitor_.size(), 0);
        for (std::size_t iter = 0; iter != iter_size(); ++iter) {
            mask.push_back(true);
            data.push_back(ess_history_[iter] / size());
            if (print_accept) {
                for (std::size_t i = 0; i != accept_history_.size(); ++i) {
                    data.push_back(accept_history_[i][iter] /
                            static_cast<double>(size()));
                    mask.push_back(true);
                }
            }
            if (print_path) {
                if (piter == path_.iter_size() ||iter != path_.index(piter)) {
                    data.push_back(0);
                    data.push_back(0);
                    mask.push_back(false);
                    mask.push_back(false);
                } else {
                    data.push_back(path_.integrand(piter));
                    data.push_back(path_.grid(piter));
                    mask.push_back(true);
                    mask.push_back(true);
                    ++piter;
                }
            }
            if (print_monitor) {
                std::size_t mm = 0;
                for (typename monitor_map_type::const_iterator
                        m = monitor_.begin(); m != monitor_.end(); ++m, ++mm) {
                    if (miter[mm] == m->second.iter_size()
                            || iter != m->second.index(miter[mm])) {
                        for (std::size_t i = 0; i != m->second.dim(); ++i) {
                            data.push_back(0);
                            mask.push_back(false);
                        }
                    } else {
                        for (std::size_t i = 0; i != m->second.dim(); ++i) {
                            data.push_back(m->second.record(i, miter[mm]));
                            mask.push_back(true);
                            ++miter[mm];
                        }
                    }
                }
            }
        }

        return std::make_pair(data, mask);
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
