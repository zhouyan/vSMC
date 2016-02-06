//============================================================================
// vSMC/include/vsmc/core/sampler.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_CORE_SAMPLER_HPP
#define VSMC_CORE_SAMPLER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/path.hpp>

#define VSMC_RUNTIME_ASSERT_CORE_SAMPLER_MONITOR_NAME(iter, map, func)        \
    VSMC_RUNTIME_ASSERT(                                                      \
        (iter != map.end()), "**Sampler::" #func "** INVALID MONITOR NAME")

#define VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(func, caller, name)          \
    VSMC_RUNTIME_ASSERT(static_cast<bool>(func),                              \
        "**Sampler::" #caller "** INVALID " #name " OBJECT")

#define VSMC_RUNTIME_WARNING_CORE_SAMPLER_INIT_BY_ITER                        \
    VSMC_RUNTIME_WARNING((!static_cast<bool>(init_)),                         \
        "**Sampler::initialize** A VALID INIT OBJECT IS SET "                 \
        "BUT INITILIALIZED BY ITERATING")

namespace vsmc
{

/// \brief SMC Sampler
/// \ingroup Core
template <typename T>
class Sampler
{
    public:
    using size_type = typename Particle<T>::size_type;
    using resample_type = typename Particle<T>::resample_type;
    using value_type = T;
    using init_type = std::function<std::size_t(Particle<T> &, void *)>;
    using move_type = std::function<std::size_t(std::size_t, Particle<T> &)>;
    using mcmc_type = std::function<std::size_t(std::size_t, Particle<T> &)>;
    using monitor_map_type = std::map<std::string, Monitor<T>>;

    /// \brief Construct a Sampler without selection of resampling method
    explicit Sampler(size_type N)
        : particle_(N)
        , init_by_iter_(false)
        , resample_threshold_(resample_threshold_never())
        , iter_num_(0)
        , path_(typename Path<T>::eval_type())
    {
        resample_scheme(Multinomial);
    }

    /// \brief Construct a Sampler with a built-in resampling scheme
    ///
    /// \details
    /// By default, resampling will be performed at every iteration.
    Sampler(size_type N, ResampleScheme scheme)
        : particle_(N)
        , init_by_iter_(false)
        , resample_threshold_(resample_threshold_always())
        , iter_num_(0)
        , path_(typename Path<T>::eval_type())
    {
        resample_scheme(scheme);
    }

    /// \brief Construct a Sampler with a user defined resampling operation
    ///
    /// \details
    /// By default, resampling will be performed at every iteration.
    Sampler(size_type N, const resample_type &res_op)
        : particle_(N)
        , init_by_iter_(false)
        , resample_threshold_(resample_threshold_always())
        , iter_num_(0)
        , path_(typename Path<T>::eval_type())
    {
        resample_scheme(res_op);
    }

    /// \brief Construct a Sampler with a built-in resampling scheme and a
    /// threshold for resampling
    Sampler(size_type N, ResampleScheme scheme, double resample_threshold)
        : particle_(N)
        , init_by_iter_(false)
        , resample_threshold_(resample_threshold)
        , iter_num_(0)
        , path_(typename Path<T>::eval_type())
    {
        resample_scheme(scheme);
    }

    /// \brief Construct a Sampler with a user defined resampling scheme and a
    /// threshold for resampling
    Sampler(
        size_type N, const resample_type &res_op, double resample_threshold)
        : particle_(N)
        , init_by_iter_(false)
        , resample_threshold_(resample_threshold)
        , iter_num_(0)
        , path_(typename Path<T>::eval_type())
    {
        resample_scheme(res_op);
    }

    /// \brief Clone the sampler system except the RNG engines
    ///
    /// \param new_rng If true, the new particle system has new-seeded RNG.
    /// Otherwise false, it is exactly the same as the current.
    Sampler<T> clone(bool new_rng) const
    {
        Sampler<T> sampler(*this);
        if (new_rng) {
            sampler.particle().rng_set().seed();
            Seed::instance().seed_rng(sampler.particle().resample_rng());
        }

        return sampler;
    }

    /// \brief Clone another sampler system except the RNG engines
    ///
    /// \param other The particle system to be cloned
    /// \param retain_rng If true, retain the current system's RNG. Otherwise,
    /// it is exactly the same as the new one.
    Sampler<T> &clone(const Sampler<T> &other, bool retain_rng)
    {
        if (this != &other) {
            particle_.clone(other.particle_, retain_rng);
            init_by_iter_ = other.init_by_iter_;
            init_ = other.init_;
            move_queue_ = other.move_queue_;
            mcmc_queue_ = other.mcmc_queue_;
            resample_op_ = other.resample_op_;
            resample_threshold_ = other.resample_threshold_;
            iter_num_ = other.iter_num_;
            size_history_ = other.size_history_;
            ess_history_ = other.ess_history_;
            resampled_history_ = other.resampled_history_;
            accept_history_ = other.accept_history_;
        }

        return *this;
    }

    Sampler<T> &clone(Sampler<T> &&other, bool retain_rng)
    {
        if (this != &other) {
            particle_.clone(std::move(other.particle_), retain_rng);
            init_by_iter_ = other.init_by_iter_;
            init_ = std::move(other.init_);
            move_queue_ = std::move(other.move_queue_);
            mcmc_queue_ = std::move(other.mcmc_queue_);
            resample_op_ = std::move(other.resample_op_);
            resample_threshold_ = other.resample_threshold_;
            iter_num_ = other.iter_num_;
            size_history_ = std::move(other.size_history_);
            ess_history_ = std::move(other.ess_history_);
            resampled_history_ = std::move(other.resampled_history_);
            accept_history_ = std::move(other.accept_history_);
        }

        return *this;
    }

    /// \brief Number of particles
    size_type size() const { return particle_.size(); }

    /// \brief Reserve space for a specified number of iterations
    void reserve(std::size_t num)
    {
        size_history_.reserve(num);
        ess_history_.reserve(num);
        resampled_history_.reserve(num);
        for (auto &a : accept_history_)
            a.reserve(num);
        if (!path_.empty())
            path_.reserve(num);
        for (auto &m : monitor_)
            if (!m.second.empty())
                m.second.reserve(num);
    }

    /// \brief Number of iterations (including initialization)
    std::size_t iter_size() const { return size_history_.size(); }

    /// \brief Current iteration number (initialization count as zero)
    std::size_t iter_num() const { return iter_num_; }

    /// \brief Force resample
    Sampler<T> &resample()
    {
        particle_.resample(resample_op_, resample_threshold_always());

        return *this;
    }

    /// \brief Set resampling method by a resample_type object
    Sampler<T> &resample_scheme(const resample_type &res_op)
    {
        resample_op_ = res_op;

        return *this;
    }

    /// \brief Set resampling method by a built-in ResampleScheme scheme
    /// name
    Sampler<T> &resample_scheme(ResampleScheme scheme)
    {
        switch (scheme) {
            case Multinomial: resample_op_ = ResampleMultinomial(); break;
            case Residual: resample_op_ = ResampleResidual(); break;
            case Stratified: resample_op_ = ResampleStratified(); break;
            case Systematic: resample_op_ = ResampleSystematic(); break;
            case ResidualStratified:
                resample_op_ = ResampleResidualStratified();
                break;
            case ResidualSystematic:
                resample_op_ = ResampleResidualSystematic();
                break;
        }

        return *this;
    }

    /// \brief Get resampling threshold
    double resample_threshold() const { return resample_threshold_; }

    /// \brief Set resampling threshold
    Sampler<T> &resample_threshold(double threshold)
    {
        resample_threshold_ = threshold;
        return *this;
    }

    /// \brief Special value of resampling threshold that indicate no
    /// resampling will be ever performed
    static double resample_threshold_never()
    {
        return -std::numeric_limits<double>::infinity();
    }

    /// \brief Special value of resampling threshold that indicate no
    /// resampling will always be performed
    static double resample_threshold_always()
    {
        return std::numeric_limits<double>::infinity();
    }

    /// \brief Get sampler size of a given iteration (initialization count as
    /// iteration zero)
    double size_history(std::size_t iter) const { return size_history_[iter]; }

    /// \brief Read sampler size history through an output iterator
    template <typename OutputIter>
    void read_size_history(OutputIter first) const
    {
        std::copy(size_history_.begin(), size_history_.end(), first);
    }

    /// \brief Get ESS of a given iteration, initialization count as iter 0
    double ess_history(std::size_t iter) const { return ess_history_[iter]; }

    /// \brief Read ESS history through an output iterator
    template <typename OutputIter>
    void read_ess_history(OutputIter first) const
    {
        std::copy(ess_history_.begin(), ess_history_.end(), first);
    }

    /// \brief Get resampling indicator of a given iteration
    bool resampled_history(std::size_t iter) const
    {
        return resampled_history_[iter];
    }

    /// \brief Read resampling indicator history through an output iterator
    template <typename OutputIter>
    void read_resampled_history(OutputIter first) const
    {
        std::copy(resampled_history_.begin(), resampled_history_.end(), first);
    }

    /// \brief Get the accept count of a given move id and the iteration
    std::size_t accept_history(std::size_t id, std::size_t iter) const
    {
        return accept_history_[id][iter];
    }

    /// \brief Read and write access to the Particle<T> object
    Particle<T> &particle() { return particle_; }

    /// \brief Read only access to the Particle<T> object
    const Particle<T> &particle() const { return particle_; }

    /// \brief Set the initialization object of type init_type
    Sampler<T> &init(const init_type &new_init)
    {
        VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(new_init, init, INIT);

        init_ = new_init;

        return *this;
    }

    /// \brief Set if initialization should use the move and mcmc queue
    ///
    /// \details
    /// If set to `false`, then the initialization step use the initialization
    /// object if it is not empty. Otherwise, it perform the same steps as the
    /// iteration step.
    Sampler<T> &init_by_iter(bool initialize_by_iterate)
    {
        init_by_iter_ = initialize_by_iterate;

        return *this;
    }

    /// \brief Set the initialization object with a type move_type object
    ///
    /// \details
    /// When called, the iteration parameter passed to this object will be 0
    /// and the `void *` parameter will be ignored.
    Sampler<T> &init_by_move(const move_type &new_init)
    {
        VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(new_init, init_by_move, MOVE);

        init_ = init_op([new_init](
            Particle<T> &particle, void *) { new_init(0, particle); });

        return *this;
    }

    /// \brief Clear the move queue
    Sampler<T> &move_queue_clear()
    {
        move_queue_.clear();
        return *this;
    }

    /// \brief Check if move queue is empty
    bool move_queue_empty() const { return move_queue_.empty(); }

    /// \brief Check the size of the move queue
    std::size_t move_queue_size() const { return move_queue_.size(); }

    /// \brief Add a new move
    Sampler<T> &move(const move_type &new_move, bool append)
    {
        VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(new_move, move, MOVE);

        if (!append)
            move_queue_.clear();
        move_queue_.push_back(new_move);

        return *this;
    }

    /// \brief Add a sequence of new moves
    template <typename InputIter>
    Sampler<T> &move(InputIter first, InputIter last, bool append)
    {
        if (!append)
            move_queue_.clear();
        while (first != last) {
            VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(*first, move, MOVE);
            move_queue_.push_back(*first);
            ++first;
        }

        return *this;
    }

    /// \brief Clear the mcmc queue
    Sampler<T> &mcmc_queue_clear()
    {
        mcmc_queue_.clear();

        return *this;
    }

    /// \brief Check if mcmc queue is empty
    bool mcmc_queue_empty() const { return mcmc_queue_.empty(); }

    /// \brief Check the size of the mcmc queue
    std::size_t mcmc_queue_size() const { return mcmc_queue_.size(); }

    /// \brief Add a new mcmc
    Sampler<T> &mcmc(const mcmc_type &new_mcmc, bool append)
    {
        VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(new_mcmc, mcmc, MCMC);

        if (!append)
            mcmc_queue_.clear();
        mcmc_queue_.push_back(new_mcmc);

        return *this;
    }

    /// \brief Add a sequence of new mcmcs
    template <typename InputIter>
    Sampler<T> &mcmc(InputIter first, InputIter last, bool append)
    {
        if (!append)
            mcmc_queue_.clear();
        while (first != last) {
            VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(*first, mcmc, MCMC);
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
    /// All histories (ESS, resampled, accept, Monitor and Path) are clared
    /// before callling the initialization object. Monitors and Path's
    /// evaluation objects are untouched.
    Sampler<T> &initialize(void *param = nullptr)
    {
        do_reset();
        do_acch();
        if (init_by_iter_) {
            VSMC_RUNTIME_WARNING_CORE_SAMPLER_INIT_BY_ITER;
            do_iter();
        } else {
            do_init(param);
        }
        do_acch();

        return *this;
    }

    /// \brief Iteration
    ///
    /// \details
    /// Moves performed first. Then ESS/N is compared to the threshold and
    /// possible resampling is performed. Then mcmcs are performed. Then
    /// monitors and Path are computed
    Sampler<T> &iterate(std::size_t num = 1)
    {
        do_acch();
        if (num > 1)
            reserve(iter_size() + num);
        for (std::size_t i = 0; i != num; ++i) {
            ++iter_num_;
            do_iter();
        }
        do_acch();

        return *this;
    }

    /// \brief Read and write access to the Path sampling monitor
    Path<T> &path() { return path_; }

    /// \brief Read only access to the Path sampling monitor
    const Path<T> &path() const { return path_; }

    /// \brief Set the Path sampling evaluation object
    Sampler<T> &path_sampling(
        const typename Path<T>::eval_type &eval, bool record_only = false)
    {
        path_.set_eval(eval, record_only);

        return *this;
    }

    /// \brief Path sampling estimate of the logarithm of normalizing
    /// constants
    /// ratio
    double path_sampling() const { return path_.log_zconst(); }

    /// \brief Add a monitor
    ///
    /// \param name The name of the monitor
    /// \param mon The new monitor to be added
    Sampler<T> &monitor(const std::string &name, const Monitor<T> &mon)
    {
        monitor_.insert(std::make_pair(name, mon));

        return *this;
    }

    /// \brief Add a monitor with an evaluation object
    ///
    /// \param name The name of the Monitor
    /// \param dim The dimension of the Monitor, i.e., the number of variables
    /// \param eval The evaluation object of type Monitor::eval_type
    /// \param record_only The Monitor only records results
    /// \param stage The stage of the Monitor
    ///
    /// \sa Monitor
    Sampler<T> &monitor(const std::string &name, std::size_t dim,
        const typename Monitor<T>::eval_type &eval, bool record_only = false,
        MonitorStage stage = MonitorMCMC)
    {
        monitor_.insert(typename monitor_map_type::value_type(
            name, Monitor<T>(dim, eval, record_only, stage)));

        return *this;
    }

    /// \brief Read and write access to a named monitor
    Monitor<T> &monitor(const std::string &name)
    {
        typename monitor_map_type::iterator iter = monitor_.find(name);

        VSMC_RUNTIME_ASSERT_CORE_SAMPLER_MONITOR_NAME(iter, monitor_, monitor);

        return iter->second;
    }

    /// \brief Read only access to a named monitor
    const Monitor<T> &monitor(const std::string &name) const
    {
        typename monitor_map_type::const_iterator citer = monitor_.find(name);

        VSMC_RUNTIME_ASSERT_CORE_SAMPLER_MONITOR_NAME(
            citer, monitor_, monitor);

        return citer->second;
    }

    /// \brief Read and write access to all monitors to the monitor_map_type
    /// object
    monitor_map_type &monitor() { return monitor_; }

    /// \brief Read only access to all monitors to the the monitor_map_type
    /// object
    const monitor_map_type &monitor() const { return monitor_; }

    /// \brief Erase a named monitor
    bool clear_monitor(const std::string &name)
    {
        return monitor_.erase(name) ==
            static_cast<typename monitor_map_type::size_type>(1);
    }

    /// \brief Erase all monitors
    Sampler<T> &clear_monitor()
    {
        monitor_.clear();

        return *this;
    }

    /// \brief The size of Sampler summary header (integer data, size etc.)
    std::size_t summary_header_size_int() const
    {
        if (iter_size() == 0)
            return 0;

        return accept_history_.size() + 2;
    }

    /// \brief The size of Sampler summary header (floating point data)
    std::size_t summary_header_size() const
    {
        if (iter_size() == 0)
            return 0;

        std::size_t header_size = 1;
        if (path_.iter_size() > 0)
            header_size += 2;
        for (const auto &m : monitor_)
            if (m.second.iter_size() > 0)
                header_size += m.second.dim();

        return header_size;
    }

    /// \brief Sampler summary header (integer data)
    template <typename OutputIter>
    void summary_header_int(OutputIter first) const
    {
        if (summary_header_size_int() == 0)
            return;

        *first++ = std::string("Size");
        *first++ = std::string("Resampled");
        for (std::size_t i = 0; i != accept_history_.size(); ++i)
            *first++ = "Accept." + internal::itos(i);
    }

    /// \brief Sampler summary header (floating point data)
    template <typename OutputIter>
    void summary_header(OutputIter first) const
    {
        if (summary_header_size() == 0)
            return;

        *first++ = std::string("ESS");
        if (path_.iter_size() > 0) {
            *first++ = std::string("Path.Integrand");
            *first++ = std::string("Path.Grid");
        }
        for (const auto &m : monitor_) {
            if (m.second.iter_size() > 0) {
                unsigned md = static_cast<unsigned>(m.second.dim());
                for (unsigned d = 0; d != md; ++d) {
                    if (m.second.name(d).empty())
                        *first++ = m.first + "." + internal::itos(d);
                    else
                        *first++ = m.second.name(d);
                }
            }
        }
    }

    /// \brief The size of Sampler summary data (integer data)
    std::size_t summary_data_size_int() const
    {
        return summary_header_size_int() * iter_size();
    }

    /// \brief The size of Sampler summary data (floating point data)
    std::size_t summary_data_size() const
    {
        return summary_header_size() * iter_size();
    }

    /// \brief Sampler summary data (integer data)
    template <MatrixOrder Order, typename OutputIter>
    void summary_data_int(OutputIter first) const
    {
        if (summary_data_size_int() == 0)
            return;

        if (Order == RowMajor)
            summary_data_row_int(first);
        if (Order == ColMajor)
            summary_data_col_int(first);
    }

    /// \brief Sampler summary data (floating point data)
    template <MatrixOrder Order, typename OutputIter>
    void summary_data(OutputIter first) const
    {
        if (summary_data_size() == 0)
            return;

        if (Order == RowMajor)
            summary_data_row(first);
        if (Order == ColMajor)
            summary_data_col(first);
    }

    /// \brief Print the history of the Sampler
    ///
    /// \param os The ostream to which the contents are printed
    /// \param sepchar The seperator of fields
    template <typename CharT, typename Traits>
    std::basic_ostream<CharT, Traits> &print(
        std::basic_ostream<CharT, Traits> &os, char sepchar = '\t') const
    {
        if (iter_size() == 0 || !os.good())
            return os;

        std::size_t nrow = iter_size();

        std::size_t ncol_int = summary_header_size_int();
        Vector<std::string> header_int(ncol_int);
        Vector<size_type> data_int(nrow * ncol_int);
        summary_header_int(header_int.begin());
        summary_data_int<RowMajor>(data_int.begin());

        std::size_t ncol = summary_header_size();
        Vector<std::string> header(ncol);
        Vector<double> data(nrow * ncol);
        summary_header(header.begin());
        summary_data<RowMajor>(data.begin());

        for (const auto &h : header_int)
            os << h << sepchar;
        for (const auto &h : header)
            os << h << sepchar;
        os << '\n';

        std::size_t offset_int = 0;
        std::size_t offset = 0;
        for (std::size_t r = 0; r != nrow; ++r) {
            for (std::size_t c = 0; c != ncol_int; ++c)
                os << data_int[offset_int++] << sepchar;
            for (std::size_t c = 0; c != ncol; ++c)
                os << data[offset++] << sepchar;
            os << '\n';
        }

        return os;
    }

    private:
    Particle<T> particle_;

    bool init_by_iter_;
    init_type init_;
    Vector<move_type> move_queue_;
    Vector<mcmc_type> mcmc_queue_;

    resample_type resample_op_;
    double resample_threshold_;

    std::size_t iter_num_;
    Vector<std::size_t> size_history_;
    Vector<double> ess_history_;
    Vector<bool> resampled_history_;
    Vector<Vector<std::size_t>> accept_history_;

    Path<T> path_;
    monitor_map_type monitor_;

    void do_acch()
    {
        if (accept_history_.empty())
            accept_history_.push_back(Vector<std::size_t>());
        std::size_t acc_size = move_queue_.size() + mcmc_queue_.size();
        if (accept_history_.size() < acc_size) {
            std::size_t diff = acc_size - accept_history_.size();
            for (std::size_t d = 0; d != diff; ++d)
                accept_history_.push_back(Vector<std::size_t>());
        }
        for (std::size_t i = 0; i != accept_history_.size(); ++i)
            accept_history_[i].resize(iter_size());
    }

    void do_reset()
    {
        size_history_.clear();
        ess_history_.clear();
        resampled_history_.clear();
        accept_history_.clear();
        path_.clear();
        for (auto &m : monitor_)
            m.second.clear();
        iter_num_ = 0;
        particle_.weight().set_equal();
    }

    void do_init(void *param)
    {
        VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(init_, initialize, INIT);
        accept_history_[0].push_back(init_(particle_, param));
        do_monitor(MonitorMove);
        do_resample();
        do_monitor(MonitorResample);
        do_mcmc(1);
        do_monitor(MonitorMCMC);
    }

    void do_iter()
    {
        std::size_t ia = do_move(0);
        do_monitor(MonitorMove);
        do_resample();
        do_monitor(MonitorResample);
        do_mcmc(ia);
        do_monitor(MonitorMCMC);
    }

    std::size_t do_move(std::size_t ia)
    {
        for (auto &m : move_queue_)
            accept_history_[ia++].push_back(m(iter_num_, particle_));

        return ia;
    }

    std::size_t do_mcmc(std::size_t ia)
    {
        for (auto &m : mcmc_queue_)
            accept_history_[ia++].push_back(m(iter_num_, particle_));

        return ia;
    }

    void do_resample()
    {
        size_history_.push_back(size());
        ess_history_.push_back(particle_.weight().ess());
        resampled_history_.push_back(
            particle_.resample(resample_op_, resample_threshold_));
    }

    void do_monitor(MonitorStage stage)
    {
        if (!path_.empty() && stage == MonitorMCMC)
            path_.eval(iter_num_, particle_);

        for (auto &m : monitor_)
            if (!m.second.empty())
                m.second.eval(iter_num_, particle_, stage);
    }

    template <typename OutputIter>
    void summary_data_row_int(OutputIter first) const
    {
        using int_type = typename std::iterator_traits<OutputIter>::value_type;
        for (std::size_t iter = 0; iter != iter_size(); ++iter) {
            *first++ = static_cast<int_type>(size_history_[iter]);
            *first++ = static_cast<int_type>(resampled_history_[iter]);
            for (std::size_t i = 0; i != accept_history_.size(); ++i)
                *first++ = static_cast<int_type>(accept_history_[i][iter]);
        }
    }

    template <typename OutputIter>
    void summary_data_col_int(OutputIter first) const
    {
        first = std::copy(size_history_.begin(), size_history_.end(), first);
        first = std::copy(
            resampled_history_.begin(), resampled_history_.end(), first);
        for (std::size_t i = 0; i != accept_history_.size(); ++i) {
            first = std::copy(
                accept_history_[i].begin(), accept_history_[i].end(), first);
        }
    }

    template <typename OutputIter>
    void summary_data_row(OutputIter first) const
    {
        double missing_data = std::numeric_limits<double>::quiet_NaN();

        std::size_t piter = 0;

        Vector<std::pair<std::size_t, const Monitor<T> *>> miter;
        for (const auto &m : monitor_)
            if (m.second.iter_size() > 0)
                miter.push_back(std::make_pair(0, &m.second));

        for (std::size_t iter = 0; iter != iter_size(); ++iter) {
            *first++ = ess_history_[iter];
            if (path_.iter_size() > 0) {
                if (piter != path_.iter_size() && iter == path_.index(piter)) {
                    *first++ = path_.integrand(piter);
                    *first++ = path_.grid(piter);
                    ++piter;
                } else {
                    *first++ = missing_data;
                    *first++ = missing_data;
                }
            }
            for (auto &m : miter) {
                std::size_t md = m.second->dim();
                if (m.first != m.second->iter_size() &&
                    iter == m.second->index(m.first)) {
                    first = std::copy_n(
                        m.second->record_data(m.first++), md, first);
                } else {
                    first = std::fill_n(first, md, missing_data);
                }
            }
        }
    }

    template <typename OutputIter>
    void summary_data_col(OutputIter first) const
    {
        double missing_data = std::numeric_limits<double>::quiet_NaN();

        first = std::copy(ess_history_.begin(), ess_history_.end(), first);
        if (path_.iter_size() > 0) {
            std::size_t piter = 0;
            for (std::size_t iter = 0; iter != iter_size(); ++iter) {
                if (piter != path_.iter_size() || iter == path_.index(piter))
                    *first++ = path_.integrand(piter++);
                else
                    *first = missing_data;
            }
            piter = 0;
            for (std::size_t iter = 0; iter != iter_size(); ++iter) {
                if (piter != path_.iter_size() || iter == path_.index(piter))
                    *first++ = path_.grid(piter++);
                else
                    *first = missing_data;
            }
        }
        for (const auto &m : monitor_) {
            if (m.second.iter_size() > 0) {
                for (std::size_t d = 0; d != m.second.dim(); ++d) {
                    std::size_t miter = 0;
                    for (std::size_t iter = 0; iter != iter_size(); ++iter) {
                        if (miter != m.second.iter_size() ||
                            iter == m.second.index(miter)) {
                            *first++ = m.second.record(d, miter++);
                        } else {
                            *first++ = missing_data;
                        }
                    }
                }
            }
        }
    }
}; // class Sampler

template <typename CharT, typename Traits, typename T>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const Sampler<T> &sampler)
{
    return sampler.print(os);
}

} // namespace vsmc

#endif // VSMC_CORE_SAMPLER_HPP
