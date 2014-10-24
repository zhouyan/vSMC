//============================================================================
// include/vsmc/core/path.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_CORE_PATH_HPP
#define VSMC_CORE_PATH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/cxx11/functional.hpp>
#include <vsmc/integrate/nintegrate_newton_cotes.hpp>
#include <vsmc/math/cblas.hpp>
#include <cmath>
#include <vector>

#define VSMC_RUNTIME_ASSERT_CORE_PATH_ITER(func) \
    VSMC_RUNTIME_ASSERT((iter < iter_size()),                                \
            ("**Path::"#func"** INVALID ITERATION NUMBER ARGUMENT"))

#define VSMC_RUNTIME_ASSERT_CORE_PATH_FUNCTOR(func, caller, name) \
    VSMC_RUNTIME_ASSERT(static_cast<bool>(func),                             \
            ("**Path::"#caller"** INVALID "#name" OBJECT"))                  \

namespace vsmc {

/// \brief Monitor for Path sampling
/// \ingroup Core
template <typename T>
class Path
{
    public :

    typedef T value_type;
    typedef cxx11::function<double (
            std::size_t, const Particle<T> &, double *)> eval_type;

    /// \brief Construct a Path with an evaluation object
    ///
    /// \param eval The evaluation object of type Path::eval_type
    /// \param record_only The Path monitor only records the integrands instead
    /// of calculating them itself
    ///
    /// A Path object is very similar to a Monitor object. It is a special case
    /// for Path sampling Monitor. The dimension of the Monitor is always one.
    /// In addition, the evaluation object returns the integration grid of the
    /// Path sampling.
    ///
    /// The evaluation object has the signature
    /// ~~~{.cpp}
    /// double eval (std::size_t iter, const Particle<T> &particle, double *integrand)
    /// ~~~
    /// where the first two arguments are passed in by the Sampler at the end
    /// of each iteration. The evaluation occurs after the possible MCMC moves.
    /// The output parameter `integrand` shall contains the results of the
    /// Path sampling integrands. The return value shall be the Path sampling
    /// integration grid.
    ///
    /// If `record_only` is true, then the Path monitor only records the
    /// integrand estimate stored in `integrand`. Otherwise the behavior is
    /// explained below,
    ///
    /// For example, say the Path sampling is computed through integration of
    /// \f$\lambda = \int_0^1 E[g_\alpha(X)]\,\mathrm{d}\alpha\f$. The integral
    /// is approximated with numerical integration at point
    /// \f$\alpha_0 = 0, \alpha_1, \dots, \alpha_T = 1\f$, then at iteration
    /// \f$t\f$, the output parameter `integrand` contains
    /// \f$(g_{\alpha_t}(X_0),\dots)\f$ and the return value is \f$\alpha_t\f$.
    explicit Path (const eval_type &eval, bool record_only = false) :
        eval_(eval), recording_(true), record_only_(record_only),
        log_zconst_(0) {}

    /// \brief The number of iterations has been recorded
    ///
    /// \sa Monitor::iter_size()
    std::size_t iter_size () const {return index_.size();}

    /// \brief Reserve space for a specified number of iterations
    void reserve (std::size_t num)
    {
        index_.reserve(num);
        integrand_.reserve(num);
        grid_.reserve(num);
    }

    /// \brief Whether the evaluation object is valid
    bool empty () const {return !static_cast<bool>(eval_);}

    /// \brief Get the iteration index of the sampler of a given monitor
    /// iteration
    ///
    /// \sa Monitor::index()
    std::size_t index (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_PATH_ITER(index);

        return index_[iter];
    }

    /// \brief Get the Path sampling integrand of a given Path iteration
    double integrand (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_PATH_ITER(integrand);

        return integrand_[iter];
    }

    /// \brief Get the Path sampling grid value of a given Path iteration
    double grid (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_PATH_ITER(grid);

        return grid_[iter];
    }

    /// \brief Read the index history through an output iterator
    ///
    /// \sa Monitor::read_index()
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        const std::size_t N = index_.size();
        const std::size_t *const iptr = &index_[0];
        for (std::size_t i = 0; i != N; ++i, ++first)
            *first = iptr[i];

        return first;
    }

    /// \brief Read the integrand history through an output iterator
    template <typename OutputIter>
    OutputIter read_integrand (OutputIter first) const
    {
        const std::size_t N = integrand_.size();
        const double *const iptr = &integrand_[0];
        for (std::size_t i = 0; i != N; ++i, ++first)
            *first = iptr[i];

        return first;
    }

    /// \brief Read the grid history through an output iterator
    template <typename OutputIter>
    OutputIter read_grid (OutputIter first) const
    {
        const std::size_t N = grid_.size();
        const double *const gptr = &grid_[0];
        for (std::size_t i = 0; i != N; ++i, ++first)
            *first = gptr[i];

        return first;
    }

    /// \brief Set a new evaluation object of type eval_type
    void set_eval (const eval_type &new_eval, bool record_only = false)
    {eval_ = new_eval; record_only_ = record_only;}

    /// Perform the evaluation for a given iteration and a Particle<T> object
    ///
    /// \sa Monitor::eval()
    void eval (std::size_t iter, const Particle<T> &particle)
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
        double *const bptr = &buffer_[0];
        const double *const wptr = particle.weight_set().weight_data();
        double grid = eval_(iter, particle, bptr);
        double integrand = math::dot(N, wptr, bptr);
        push_back(iter, grid, integrand);
    }

    /// \brief Get the nomralizing constants ratio estimates
    double zconst () const {return std::exp(log_zconst_);}

    /// \brief Get the logarithm nomralizing constants ratio estimates
    double log_zconst () const {return log_zconst_;}

    /// \brief Clear all records of the index and integrations
    void clear ()
    {
        log_zconst_ = 0;
        index_.clear();
        integrand_.clear();
        grid_.clear();
    }

    /// \brief Whether the Path is actively recording restuls
    bool recording () const {return recording_;}

    /// \brief Turn on the recording
    void turn_on () {recording_ = true;}

    /// \brief Turn off the recording
    void turn_off () {recording_ = false;}

    private :

    eval_type eval_;
    bool recording_;
    bool record_only_;
    double log_zconst_;
    std::vector<std::size_t> index_;
    std::vector<double> integrand_;
    std::vector<double> grid_;
    std::vector<double> buffer_;

    void push_back (std::size_t iter, double grid, double integrand)
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
