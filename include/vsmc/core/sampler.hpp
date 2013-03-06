#ifndef VSMC_CORE_SAMPLER_HPP
#define VSMC_CORE_SAMPLER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/path.hpp>
#include <vsmc/utility/backup.hpp>

namespace vsmc {

/// \brief SMC Sampler
/// \ingroup Core
template <typename T>
class Sampler
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef typename Particle<T>::resample_type resample_type;
    typedef T value_type;
    typedef cxx11::function<std::size_t (Particle<T> &, void *)> init_type;
    typedef cxx11::function<std::size_t (std::size_t, Particle<T> &)>
        move_type;
    typedef cxx11::function<std::size_t (std::size_t, Particle<T> &)>
        mcmc_type;
    typedef std::map<std::string, Monitor<T> > monitor_map_type;

    explicit Sampler (size_type N,
            ResampleScheme scheme = Stratified,
            double resample_threshold = 0.5) :
        resample_threshold_(resample_threshold), particle_(N), iter_num_(0),
        path_(typename Path<T>::eval_type()), show_(false)
    {resample_scheme(scheme);}

    explicit Sampler (size_type N,
            const resample_type &res_op,
            double resample_threshold = 0.5) :
        resample_threshold_(resample_threshold), particle_(N), iter_num_(0)
    {resample_scheme(res_op);}

    /// \brief Number of particles
    size_type size () const {return particle_.size();}

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
    std::size_t iter_size () const {return ess_history_.size();}

    /// \brief Set resampling method by a resample_type object
    Sampler<T> &resample_scheme (const resample_type &res_op)
    {resample_op_ = res_op; return *this;}

    /// \brief Set resampling method by a built-in ResampleScheme scheme
    /// name
    Sampler<T> &resample_scheme (ResampleScheme scheme)
    {
        switch (scheme) {
            case Multinomial :
                resample_op_ = Resample<cxx11::integral_constant<
                    ResampleScheme, Multinomial> >();
                break;
            case Residual :
                resample_op_ = Resample<cxx11::integral_constant<
                    ResampleScheme, Residual> >();
                break;
            case Stratified :
                resample_op_ = Resample<cxx11::integral_constant<
                    ResampleScheme, Stratified> >();
                break;
            case Systematic :
                resample_op_ = Resample<cxx11::integral_constant<
                    ResampleScheme, Systematic> >();
                break;
            case ResidualStratified :
                resample_op_ = Resample<cxx11::integral_constant<
                    ResampleScheme, ResidualStratified> >();
                break;
            case ResidualSystematic :
                resample_op_ = Resample<cxx11::integral_constant<
                    ResampleScheme, ResidualSystematic> >();
                break;
            default :
                break;
        }

        return *this;
    }

    /// \brief Get resampling threshold
    double resample_threshold () const {return resample_threshold_;}

    /// \brief Set resampling threshold
    Sampler<T> &resample_threshold (double threshold)
    {resample_threshold_ = threshold; return *this;}

    /// \brief Get ESS of a given iteration, initialization count as iter 0
    double ess_history (std::size_t iter) const {return ess_history_[iter];}

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
    {return resampled_history_[iter];}

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
    {return accept_history_[iter].size();}

    /// \brief Get the accept count of a given move id and the iteration
    ///
    /// \details
    /// The total number of move can be get through move_num(). The first move
    /// performed (either a move or a mcmc) has `id` 0 and so on.
    std::size_t accept_history (std::size_t id, std::size_t iter) const
    {return accept_history_[id][iter];}

    /// \brief Read and write access to the Particle<T> object
    Particle<T> &particle () {return particle_;}

    /// \brief Read only access to the Particle<T> object
    const Particle<T> &particle () const {return particle_;}

    /// \brief Set the initialization object of type init_type
    Sampler<T> &init (const init_type &new_init)
    {
        VSMC_RUNTIME_ASSERT_FUNCTOR(new_init, Sampler::init, Initialize);

        init_ = new_init;

        return *this;
    }

    /// \brief Clear the move queue
    Sampler<T> &move_queue_clear () {move_queue_.clear(); return *this;}

    /// \brief Check if move queue is empty
    bool move_queue_empty () const {return move_queue_.empty();}

    /// \brief Check the size of the move queue
    std::size_t move_queue_size () const {return move_queue_.size();}

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
    Sampler<T> &mcmc_queue_clear () {mcmc_queue_.clear(); return *this;}

    /// \brief Check if mcmc queue is empty
    bool mcmc_queue_empty () const {return mcmc_queue_.empty();}

    /// \brief Check the size of the mcmc queue
    std::size_t mcmc_queue_size () const {return mcmc_queue_.size();}

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

    /// \brief Try iteration
    ///
    /// \details
    /// This method is similar to iterate() except that it will try (its best)
    /// to save the sampler itself and restore itself if an exception condition
    /// raised during the iterations.
    Sampler<T> &try_iterate (std::size_t num = 1)
    {
        Backup<Sampler<T> > backup(this);
        try {
            iterate(num);
        } catch (...) {
            backup.restore(this);
            throw;
        }

        return *this;
    }

    /// \brief Read and write access to the Path sampling monitor
    Path<T> &path () {return path_;}

    /// \brief Read only access to the Path sampling monitor
    const Path<T> &path () const {return path_;}

    /// \brief Set the Path sampling evaluation object
    Sampler<T> &path_sampling (const typename Path<T>::eval_type &eval)
    {path_.set_eval(eval); return *this;}

    /// \brief Path sampling estimate of the logarithm of normalizing constants
    /// ratio
    double path_sampling () const {return path_.zconst();}

    /// \brief Add a monitor
    ///
    /// \param name The name of the monitor
    /// \param mon The new monitor to be added
    Sampler<T> &monitor (const std::string &name, const Monitor<T> &mon)
    {monitor_.insert(std::make_pair(name, mon)); return *this;}

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
    monitor_map_type &monitor () {return monitor_;}

    /// \brief Read only access to all monitors to the the monitor_map_type
    /// object
    const monitor_map_type &monitor () const {return monitor_;}

    /// \brief Erase a named monitor
    bool clear_monitor (const std::string &name)
    {
        return monitor_.erase(name) ==
            static_cast<typename monitor_map_type::size_type>(1);
    }

    /// \brief Erase all monitors
    Sampler<T> &clear_monitor () {monitor_.clear(); return *this;}

    /// \brief Set if the sampler shall print dots at each iteration
    Sampler<T> &show_progress (bool show) {show_ = show; return *this;}

    /// \brief The size of Sampler summary header
    std::size_t summary_header_size () const
    {
        if (iter_size() == 0)
            return 0;

        std::size_t header_size = 1; // ESS
        header_size += accept_history_.size();
        if (path_.iter_size() > 0)
            header_size +=  2;
        for (typename monitor_map_type::const_iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            if (m->second.iter_size() > 0)
                header_size += m->second.dim();
        }

        return header_size;
    }

    /// \brief Sampler summary header
    template <typename OutputIter>
    OutputIter summary_header (OutputIter first) const
    {
        if (summary_header_size() == 0)
            return first;

        *first = std::string("ESS");
        ++first;

        char acc_name[32];
        unsigned accd = static_cast<unsigned>(accept_history_.size());
        for (unsigned i = 0; i != accd; ++i) {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4996)
#endif
            std::sprintf(acc_name, "Accept.%u", i + 1);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
            *first = std::string(acc_name);
            ++first;
        }

        if (path_.iter_size() > 0) {
            *first = std::string("Path.Integrand");
            ++first;
            *first = std::string("Path.Grid");
            ++first;
        }

        for (typename monitor_map_type::const_iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            if (m->second.iter_size() > 0) {
                char *mon_name = new char[m->first.size() + 32];
                unsigned mond = static_cast<unsigned>(m->second.dim());
                for (unsigned i = 0; i != mond; ++i, ++first) {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4996)
#endif
                    std::sprintf(mon_name, "%s.%u", m->first.c_str(), i + 1);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
                    *first = std::string(mon_name);
                }
                delete [] mon_name;
            }
        }

        return first;
    }

    /// \brief The size of Sampler summary data
    std::size_t summary_data_size () const
    {return summary_header_size() * iter_size();}

    /// \brief Sampler summary data
    ///
    /// \param order The order of the data, row or column major
    /// \param first The beginning of the output
    template <typename OutputIter>
    OutputIter summary_data (MatrixOrder order, OutputIter first) const
    {
        if (summary_data_size() == 0)
            return first;

        VSMC_RUNTIME_ASSERT_MATRIX_ORDER(order, Sampler::summary_data);

        if (order == RowMajor)
            first = summary_data_row(first);

        if (order == ColMajor)
            first = summary_data_col(first);

        return first;
    }

    /// \brief Print the history of the Sampler
    ///
    /// \param os The ostream to which the contents are printed
    /// \param sampler_id The ID of the sampler
    template<typename OutputStream>
    OutputStream &print (OutputStream &os = std::cout,
            std::size_t sampler_id = 0) const
    {
        std::size_t var_num = summary_header_size();
        std::vector<std::string> header(var_num);
        std::vector<double> data(var_num * iter_size());
        summary_header(header.begin());
        summary_data(RowMajor, data.begin());

        os << "Sampler.ID Iteration Resampled";
        for (std::size_t i = 0; i != header.size(); ++i)
            os << ' ' << header[i];
        if (iter_size() > 0)
            os << '\n';

        std::size_t data_offset = 0;
        for (std::size_t iter = 0; iter != iter_size(); ++iter) {
            os << sampler_id;
            os << ' ' << iter;
            os << ' ' << resampled_history_[iter];
            for (std::size_t i = 0; i != var_num; ++i)
                os << ' ' << data[data_offset++];
            os << '\n';
        }

        return os;
    }

    private :

    init_type init_;
    std::vector<move_type> move_queue_;
    std::vector<mcmc_type> mcmc_queue_;

    resample_type resample_op_;
    double resample_threshold_;

    Particle<T> particle_;
    std::size_t iter_num_;
    std::vector<double> ess_history_;
    std::vector<bool> resampled_history_;
    std::vector<std::vector<std::size_t> > accept_history_;

    Path<T> path_;
    monitor_map_type monitor_;

    bool show_;

    void do_resampling ()
    {
        bool resampled = particle_.resample(resample_op_, resample_threshold_);
        ess_history_.push_back(particle_.ess());
        resampled_history_.push_back(resampled);
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

    template <typename OutputIter>
    OutputIter summary_data_row (OutputIter first) const
    {
        double missing_data = std::numeric_limits<double>::quiet_NaN();

        std::size_t piter = 0;
        std::vector<std::size_t> miter(monitor_.size(), 0);
        for (std::size_t iter = 0; iter != iter_size(); ++iter) {
            *first = ess_history_[iter] / size();
            ++first;
            for (std::size_t i = 0; i != accept_history_.size(); ++i) {
                *first = accept_history_[i][iter] /
                    static_cast<double>(size());
                ++first;
            }
            if (path_.iter_size() > 0) {
                if (piter == path_.iter_size() ||iter != path_.index(piter)) {
                    *first = missing_data;
                    ++first;
                    *first = missing_data;
                    ++first;
                } else {
                    *first = path_.integrand(piter);
                    ++first;
                    *first = path_.grid(piter);
                    ++first;
                    ++piter;
                }
            }
            std::size_t mm = 0;
            for (typename monitor_map_type::const_iterator
                    m = monitor_.begin(); m != monitor_.end(); ++m, ++mm) {
                if (m->second.iter_size() > 0) {
                    if (miter[mm] == m->second.iter_size()
                            || iter != m->second.index(miter[mm])) {
                        for (std::size_t i = 0; i != m->second.dim();
                                ++i, ++first)
                            *first = missing_data;
                    } else {
                        for (std::size_t i = 0; i != m->second.dim();
                                ++i, ++first)
                            *first = m->second.record(i, miter[mm]);
                        ++miter[mm];
                    }
                }
            }
        }

        return first;
    }

    template <typename OutputIter>
    OutputIter summary_data_col (OutputIter first) const
    {
        double missing_data = std::numeric_limits<double>::quiet_NaN();

        for (std::size_t iter = 0; iter != iter_size(); ++iter, ++first)
            *first = ess_history_[iter] / size();
        for (std::size_t i = 0; i != accept_history_.size(); ++i) {
            for (std::size_t iter = 0; iter != iter_size(); ++iter, ++first)
                *first = accept_history_[i][iter] /
                    static_cast<double>(size());
        }
        if (path_.iter_size() > 0) {
            std::size_t piter;
            piter = 0;
            for (std::size_t iter = 0; iter != iter_size(); ++iter, ++first) {
                if (piter == path_.iter_size() ||iter != path_.index(piter)) {
                    *first = missing_data;
                } else {
                    *first = path_.integrand(piter);
                    ++piter;
                }
            }
            piter = 0;
            for (std::size_t iter = 0; iter != iter_size(); ++iter, ++first) {
                if (piter == path_.iter_size() ||iter != path_.index(piter)) {
                    *first = missing_data;
                } else {
                    *first = path_.grid(piter);
                    ++piter;
                }
            }
        }
        for (typename monitor_map_type::const_iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            if (m->second.iter_size() > 0) {
                for (std::size_t i = 0; i != m->second.dim(); ++i) {
                    std::size_t miter = 0;
                    for (std::size_t iter = 0; iter != iter_size();
                            ++iter, ++first) {
                        if (miter == m->second.iter_size()
                                || iter != m->second.index(miter)) {
                            *first = missing_data;
                        } else {
                            *first = m->second.record(i, miter);
                            ++miter;
                        }
                    }
                }
            }
        }

        return first;
    }
}; // class Sampler

/// \brief Print the Sampler
/// \ingroup Core
template<typename OutputStream, typename T>
inline OutputStream &operator<< (OutputStream &os, const Sampler<T> &sampler)
{return sampler.print(os);}

} // namespace vsmc

#endif // VSMC_CORE_SAMPLER_HPP
