//============================================================================
// vSMC/include/vsmc/core/sampler.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#define VSMC_RUNTIME_ASSERT_CORE_SAMPLER_MONITOR_NAME(iter, map, func)        \
    VSMC_RUNTIME_ASSERT(                                                      \
        (iter != map.end()), "**Sampler::" #func "** INVALID MONITOR NAME")

#define VSMC_RUNTIME_ASSERT_CORE_SAMPLER_FUNCTOR(func, caller, name)          \
    VSMC_RUNTIME_ASSERT(                                                      \
        (func), "**Sampler::" #caller "** INVALID " #name " OBJECT")

namespace vsmc
{

/// \brief Sampler evaluaiton stages
/// \ingroup Core
enum SamplerStage {
    SamplerInit = 1 << 0, ///< Evaluation at initialization before resampling
    SamplerMove = 1 << 1, ///< Evaluation at iteration before resampling
    SamplerMCMC = 1 << 2  ///< Evaluation at after resampling
};                        // enum SamplerStage

inline constexpr SamplerStage operator&(SamplerStage s1, SamplerStage s2)
{
    return static_cast<SamplerStage>(
        static_cast<int>(s1) & static_cast<int>(s2));
}

inline constexpr SamplerStage operator|(SamplerStage s1, SamplerStage s2)
{
    return static_cast<SamplerStage>(
        static_cast<int>(s1) | static_cast<int>(s2));
}

inline constexpr SamplerStage operator^(SamplerStage s1, SamplerStage s2)
{
    return static_cast<SamplerStage>(
        static_cast<int>(s1) ^ static_cast<int>(s2));
}

inline constexpr SamplerStage operator~(SamplerStage s)
{
    return static_cast<SamplerStage>(~static_cast<int>(s));
}

inline SamplerStage &operator&=(SamplerStage &s1, SamplerStage s2)
{
    return s1 = s1 & s2;
}

inline SamplerStage &operator|=(SamplerStage &s1, SamplerStage s2)
{
    return s1 = s1 | s2;
}

inline SamplerStage &operator^=(SamplerStage &s1, SamplerStage s2)
{
    return s1 = s1 ^ s2;
}

/// \brief SMC Sampler
/// \ingroup Core
template <typename T>
class Sampler
{
    public:
    using size_type = typename Particle<T>::size_type;
    using eval_type = std::function<void(std::size_t, Particle<T> &)>;

    /// \brief Construct a Sampler
    ///
    /// \details
    /// All arguments are passed to the constructor of Particle
    template <typename... Args>
    explicit Sampler(Args &&... args)
        : particle_(std::forward<Args>(args)...)
        , iter_num_(0)
        , resample_threshold_(resample_threshold_never())
    {
    }

    /// \brief Clone the Sampler except the RNG engines
    Sampler<T> clone() const
    {
        Sampler<T> sampler(*this);
        sampler.particle().rng_set().seed();
        Seed::instance()(sampler.particle().rng());

        return sampler;
    }

    /// \brief Number of particles
    size_type size() const { return particle_.size(); }

    /// \brief Reserve space for a specified number of iterations
    void reserve(std::size_t num)
    {
        num += iter_num_;
        size_history_.reserve(num);
        ess_history_.reserve(num);
        resampled_history_.reserve(num);
        for (auto &m : monitor_)
            m.second.reserve(num);
    }

    /// \brief Number of iterations (including initialization)
    std::size_t iter_size() const { return size_history_.size(); }

    /// \brief Current iteration number (initialization count as zero)
    std::size_t iter_num() const { return iter_num_; }

    /// \brief Reset a sampler
    void reset()
    {
        clear();
        eval_clear();
    }

    /// \brief Clear all history
    void clear()
    {
        iter_num_ = 0;
        particle_.weight().set_equal();
        for (auto &m : monitor_)
            m.second.clear();

        size_history_.clear();
        ess_history_.clear();
        resampled_history_.clear();
    }

    /// \brief Clear the evaluation sequence
    void eval_clear() { eval_.clear(); }

    /// \brief Add a new evaluation object
    Sampler<T> &eval(
        const eval_type &new_eval, SamplerStage stage, bool append = true)
    {
        if (!append)
            eval_.clear();
        eval_.push_back(std::make_pair(new_eval, stage));

        return *this;
    }

    /// \brief Set resampling method by a built-in ResampleScheme scheme
    /// name
    Sampler<T> &resample_method(
        ResampleScheme scheme, double threshold = resample_threshold_always())
    {
        switch (scheme) {
            case Multinomial:
                resample_eval_ = ResampleEval<T>(ResampleMultinomial());
                break;
            case Residual:
                resample_eval_ = ResampleEval<T>(ResampleResidual());
                break;
            case Stratified:
                resample_eval_ = ResampleEval<T>(ResampleStratified());
                break;
            case Systematic:
                resample_eval_ = ResampleEval<T>(ResampleSystematic());
                break;
            case ResidualStratified:
                resample_eval_ = ResampleEval<T>(ResampleResidualStratified());
                break;
            case ResidualSystematic:
                resample_eval_ = ResampleEval<T>(ResampleResidualSystematic());
                break;
        }
        resample_threshold(threshold);

        return *this;
    }

    /// \brief Set resampling method by a `eval_type` object
    Sampler<T> &resample_method(const eval_type &res_eval,
        double threshold = resample_threshold_always())
    {
        resample_eval_ = res_eval;
        resample_threshold(threshold);

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

    /// \brief Add a monitor
    ///
    /// \param name The name of the monitor
    /// \param mon The new monitor to be added
    Sampler<T> &monitor(const std::string &name, const Monitor<T> &mon)
    {
        monitor_.insert(std::make_pair(name, mon));

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

    /// \brief Erase a named monitor
    void monitor_clear(const std::string &name) { monitor_.erase(name); }

    /// \brief Erase all monitors
    void monitor_clear() { monitor_.clear(); }

    /// \brief Initialization
    Sampler<T> &initialize()
    {
        do_initialize();

        return *this;
    }

    /// \brief Iteration
    Sampler<T> &iterate(std::size_t num = 1)
    {
        if (num > 1)
            reserve(num);
        for (std::size_t i = 0; i != num; ++i)
            do_iterate();

        return *this;
    }

    /// \brief Read and write access to the Particle<T> object
    Particle<T> &particle() { return particle_; }

    /// \brief Read only access to the Particle<T> object
    const Particle<T> &particle() const { return particle_; }

    /// \brief Get sampler size of a given iteration (initialization count as
    /// iteration zero)
    size_type size_history(std::size_t iter) const
    {
        return size_history_[iter];
    }

    /// \brief Read sampler size history through an output iterator
    template <typename OutputIter>
    OutputIter read_size_history(OutputIter first) const
    {
        return std::copy(size_history_.begin(), size_history_.end(), first);
    }

    /// \brief Get ESS of a given iteration, (initialization count as iter
    /// zero)
    double ess_history(std::size_t iter) const { return ess_history_[iter]; }

    /// \brief Read ESS history through an output iterator
    template <typename OutputIter>
    OutputIter read_ess_history(OutputIter first) const
    {
        return std::copy(ess_history_.begin(), ess_history_.end(), first);
    }

    /// \brief Get resampling indicator of a given iteration
    bool resampled_history(std::size_t iter) const
    {
        return resampled_history_[iter];
    }

    /// \brief Read resampling indicator history through an output iterator
    template <typename OutputIter>
    OutputIter read_resampled_history(OutputIter first) const
    {
        return std::copy(
            resampled_history_.begin(), resampled_history_.end(), first);
    }

    /// \brief Summary of sampler history
    std::map<std::string, Vector<double>> summary() const
    {
        const double missing_data = std::numeric_limits<double>::quiet_NaN();

        std::map<std::string, Vector<double>> df;
        Vector<double> data(iter_size());

        std::copy(size_history_.begin(), size_history_.end(), data.begin());
        df["Size"] = data;

        std::copy(resampled_history_.begin(), resampled_history_.end(),
            data.begin());
        df["Resampled"] = data;

        df["ESS"] = ess_history_;

        for (const auto &m : monitor_) {
            if (m.second.iter_size() > 0) {
                for (std::size_t d = 0; d != m.second.dim(); ++d) {
                    std::size_t miter = 0;
                    for (std::size_t i = 0; i != iter_size(); ++i) {
                        if (miter != m.second.iter_size() &&
                            i == m.second.index(miter)) {
                            data[i] = m.second.record(d, miter++);
                        } else {
                            data[i] = missing_data;
                        }
                    }
                    if (m.second.name(d).empty())
                        df[m.first + "." + std::to_string(d)] = data;
                    else
                        df[m.second.name(d)] = data;
                }
            }
        }

        return df;
    }

    /// \brief Print the history of the Sampler
    ///
    /// \param os The ostream to which the contents are printed
    /// \param sepchar The seperator of fields
    template <typename CharT, typename Traits>
    std::basic_ostream<CharT, Traits> &print(
        std::basic_ostream<CharT, Traits> &os, char sepchar = '\t') const
    {
        if (!os || iter_size() == 0)
            return os;

        const std::map<std::string, Vector<double>> df = summary();

        for (const auto &v : df)
            os << v.first << sepchar;
        os << '\n';
        for (std::size_t i = 0; i != iter_size(); ++i) {
            for (const auto &v : df)
                os << v.second[i] << sepchar;
            os << '\n';
        }

        return os;
    }

    private:
    using monitor_map_type = std::map<std::string, Monitor<T>>;

    Particle<T> particle_;
    std::size_t iter_num_;

    Vector<std::pair<eval_type, SamplerStage>> eval_;
    eval_type resample_eval_;
    double resample_threshold_;

    Vector<size_type> size_history_;
    Vector<double> ess_history_;
    Vector<bool> resampled_history_;
    monitor_map_type monitor_;

    void do_initialize()
    {
        clear();
        do_init();
        do_common();
    }

    void do_iterate()
    {
        ++iter_num_;
        do_move();
        do_common();
    }

    void do_common()
    {
        do_monitor(MonitorMove);
        do_resample(resample_threshold_);
        do_monitor(MonitorResample);
        do_mcmc();
        do_monitor(MonitorMCMC);
    }

    void do_init()
    {
        for (auto &e : eval_)
            if ((e.second & SamplerInit) != 0)
                e.first(iter_num_, particle_);
    }

    void do_move()
    {
        for (auto &e : eval_)
            if ((e.second & SamplerMove) != 0)
                e.first(iter_num_, particle_);
    }

    void do_mcmc()
    {
        for (auto &e : eval_)
            if ((e.second & SamplerMCMC) != 0)
                e.first(iter_num_, particle_);
    }

    void do_resample(double threshold)
    {
        size_history_.push_back(size());
        ess_history_.push_back(particle_.weight().ess());

        if (ess_history_.back() < size() * threshold) {
            resampled_history_.push_back(true);
            resample_eval_(iter_num_, particle_);
        } else {
            resampled_history_.push_back(false);
        }
    }

    void do_monitor(MonitorStage stage)
    {
        for (auto &m : monitor_)
            if (!m.second.empty())
                m.second(iter_num_, particle_, stage);
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
