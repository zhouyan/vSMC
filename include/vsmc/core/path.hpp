//============================================================================
// vSMC/include/vsmc/core/path.hpp
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

#ifndef VSMC_CORE_PATH_HPP
#define VSMC_CORE_PATH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/aligned_memory.hpp>

#define VSMC_RUNTIME_ASSERT_CORE_PATH_ITER(func)                             \
    VSMC_RUNTIME_ASSERT((iter < iter_size()),                                \
        ("**Path::" #func "** INVALID ITERATION NUMBER ARGUMENT"))

#define VSMC_RUNTIME_ASSERT_CORE_PATH_FUNCTOR(func, caller, name)            \
    VSMC_RUNTIME_ASSERT(static_cast<bool>(func),                             \
        ("**Path::" #caller "** INVALID " #name " OBJECT"))

namespace vsmc
{

/// \brief Monitor for Path sampling
/// \ingroup Core
template <typename T>
class Path
{
    public:
    typedef T value_type;
    typedef std::function<double(std::size_t, const Particle<T> &, double *)>
        eval_type;

    /// \brief Construct a Path with an evaluation object
    ///
    /// \param eval The evaluation object of type Path::eval_type
    /// \param record_only The Path monitor only records the integrands
    /// instead
    /// of calculating them itself
    ///
    /// A Path object is very similar to a Monitor object. It is a special
    /// case
    /// for Path sampling Monitor. The dimension of the Monitor is always one.
    /// In addition, the evaluation object returns the integration grid of the
    /// Path sampling.
    ///
    /// The evaluation object has the signature
    /// ~~~{.cpp}
    /// double eval (std::size_t iter, const Particle<T> &particle, double
    /// *integrand)
    /// ~~~
    /// where the first two arguments are passed in by the Sampler at the end
    /// of each iteration. The evaluation occurs after the possible MCMC
    /// moves.
    /// The output parameter `integrand` shall contains the results of the
    /// Path sampling integrands. The return value shall be the Path sampling
    /// integration grid.
    ///
    /// If `record_only` is true, then the Path monitor only records the
    /// integrand estimate stored in `integrand`. Otherwise the behavior is
    /// explained below,
    ///
    /// For example, say the Path sampling is computed through integration of
    /// \f$\lambda = \int_0^1 E[g_\alpha(X)]\,\mathrm{d}\alpha\f$. The
    /// integral
    /// is approximated with numerical integration at point
    /// \f$\alpha_0 = 0, \alpha_1, \dots, \alpha_T = 1\f$, then at iteration
    /// \f$t\f$, the output parameter `integrand` contains
    /// \f$(g_{\alpha_t}(X_0),\dots)\f$ and the return value is
    /// \f$\alpha_t\f$.
    explicit Path(const eval_type &eval, bool record_only = false)
        : eval_(eval)
        , recording_(true)
        , record_only_(record_only)
        , log_zconst_(0)
    {
    }

    /// \brief The number of iterations has been recorded
    ///
    /// \sa Monitor::iter_size()
    std::size_t iter_size() const { return index_.size(); }

    /// \brief Reserve space for a specified number of iterations
    void reserve(std::size_t num)
    {
        index_.reserve(num);
        integrand_.reserve(num);
        grid_.reserve(num);
    }

    /// \brief Whether the evaluation object is valid
    bool empty() const { return !static_cast<bool>(eval_); }

    /// \brief Get the iteration index of the sampler of a given monitor
    /// iteration
    ///
    /// \sa Monitor::index()
    std::size_t index(std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_PATH_ITER(index);

        return index_[iter];
    }

    /// \brief Get the Path sampling integrand of a given Path iteration
    double integrand(std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_PATH_ITER(integrand);

        return integrand_[iter];
    }

    /// \brief Get the Path sampling grid value of a given Path iteration
    double grid(std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_PATH_ITER(grid);

        return grid_[iter];
    }

    /// \brief Read only access to the raw data of the index vector
    const std::size_t *index_data() const { return index_.data(); }

    /// \brief Read only access to the raw data of the integrand vector
    const std::size_t *integrand_data() const { return integrand_.data(); }

    /// \brief Read only access to the raw data of the grid vector
    const std::size_t *grid_data() const { return grid_.data(); }

    /// \brief Read the index history through an output iterator
    ///
    /// \sa Monitor::read_index()
    template <typename OutputIter>
    void read_index(OutputIter first) const
    {
        std::copy(index_.begin(), index_.end(), first);
    }

    /// \brief Read the integrand history through an output iterator
    template <typename OutputIter>
    void read_integrand(OutputIter first) const
    {
        std::copy(integrand_.begin(), integrand_.end(), first);
    }

    /// \brief Read the grid history through an output iterator
    template <typename OutputIter>
    void read_grid(OutputIter first) const
    {
        std::copy(grid_.begin(), grid_.end(), first);
    }

    /// \brief Set a new evaluation object of type eval_type
    void set_eval(const eval_type &new_eval, bool record_only = false)
    {
        eval_ = new_eval;
        record_only_ = record_only;
    }

    /// Perform the evaluation for a given iteration and a Particle<T> object
    ///
    /// \sa Monitor::eval()
    void eval(std::size_t iter, const Particle<T> &particle)
    {
        if (!recording_)
            return;

        VSMC_RUNTIME_ASSERT_CORE_PATH_FUNCTOR(eval_, eval, EVALUATION);

        if (record_only_) {
            double integrand = 0;
            double grid = eval_(iter, particle, &integrand);
            push_back(iter, grid, integrand);

            return;
        }

        const std::size_t N = static_cast<std::size_t>(particle.size());
        buffer_.resize(N);
        double grid = eval_(iter, particle, buffer_.data());
        double integrand =
            math::dot(N, particle.weight_set().weight_data(), buffer_.data());
        push_back(iter, grid, integrand);
    }

    /// \brief Get the nomralizing constants ratio estimates
    double zconst() const { return std::exp(log_zconst_); }

    /// \brief Get the logarithm nomralizing constants ratio estimates
    double log_zconst() const { return log_zconst_; }

    /// \brief Clear all records of the index and integrations
    void clear()
    {
        log_zconst_ = 0;
        index_.clear();
        integrand_.clear();
        grid_.clear();
    }

    /// \brief Whether the Path is actively recording restuls
    bool recording() const { return recording_; }

    /// \brief Turn on the recording
    void turn_on() { recording_ = true; }

    /// \brief Turn off the recording
    void turn_off() { recording_ = false; }

    private:
    eval_type eval_;
    bool recording_;
    bool record_only_;
    double log_zconst_;
    std::vector<std::size_t> index_;
    std::vector<double, AlignedAllocator<double>> integrand_;
    std::vector<double, AlignedAllocator<double>> grid_;
    std::vector<double, AlignedAllocator<double>> buffer_;

    void push_back(std::size_t iter, double grid, double integrand)
    {
        index_.push_back(iter);
        grid_.push_back(grid);
        integrand_.push_back(integrand);
        if (iter_size() > 1) {
            std::size_t i = iter_size() - 1;
            log_zconst_ += 0.5 * (grid_[i] - grid_[i - 1]) *
                (integrand_[i] + integrand_[i - 1]);
        }
    }
}; // class PathSampling

} // namespace vsmc

#endif // VSMC_CORE_PATH_HPP
