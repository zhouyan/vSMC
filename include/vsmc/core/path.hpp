#ifndef VSMC_CORE_PATH_HPP
#define VSMC_CORE_PATH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cblas_op.hpp>

namespace vsmc {

/// \brief Monitor for path sampling
/// \ingroup Core
template <typename T>
class Path
{
    public :

    typedef T value_type;
    typedef typename traits::DDOTTypeTrait<T>::type ddot_type;
    typedef cxx11::function<double (
            unsigned, const Particle<T> &, double *)> eval_type;

    explicit Path (const eval_type &eval = eval_type()) : eval_(eval) {}

    Path (const Path<T> &other) :
        eval_(other.eval_),
        index_(other.index_), integrand_(other.integrand_),
        width_(other.width_), grid_(other.grid_) {}

    Path<T> &operator= (const Path<T> &other)
    {
        if (&other != this) {
            eval_      = other.eval_;
            index_     = other.index_;
            integrand_ = other.integrand_;
            width_     = other.width_;
            grid_      = other.grid_;
        }

        return *this;
    }

    /// \brief The number of iterations has been recorded
    ///
    /// \note This is not necessarily the same as Sampler<T>::iter_size. For
    /// example, a path sampling monitor can be set only after a certain time
    /// point of the sampler's iterations.
    unsigned iter_size () const
    {
        return static_cast<unsigned>(index_.size());
    }

    /// \brief Whether the evaluation object is valid
    VSMC_EXPLICIT_OPERATOR operator bool () const
    {
        return bool(eval_);
    }

    /// \brief Get the iteration index of the sampler of a given monitor
    /// iteration
    ///
    /// \details
    /// For example, if a path sampling monitor is only set at the sampler's
    /// iteration `siter`. Then index(0) will be `siter` and so on. If the path
    /// sampling monitor is set before the sampler's initialization and
    /// continued to be evaluated during the iterations, then iter(iter) shall
    /// just be `iter`.
    unsigned index (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::index** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return index_[iter];
    }

    /// \brief Get the path sampling integrand of a given path iteration
    double integrand (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::integrand** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return integrand_[iter];
    }

    /// \brief Get the path sampling width of a given path iteration
    double width (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::width** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return width_[iter];
    }

    /// \brief Get the path sampling grid value of a given path iteration
    ///
    /// \details
    /// This shall be sum of width's from zero up to the given iteration
    double grid (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::grid** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return grid_[iter];
    }

    /// \brief Read the index history through an output iterator
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        return std::copy(index_.begin(), index_.end(), first);
    }

    /// \brief Read the integrand history through an output iterator
    template <typename OutputIter>
    OutputIter read_integrand (OutputIter first) const
    {
        return std::copy(integrand_.begin(), integrand_.end(), first);
    }

    /// \brief Read the width history through an output iterator
    template <typename OutputIter>
    OutputIter read_width (OutputIter first) const
    {
        return std::copy(width_.begin(), width_.end(), first);
    }

    /// \brief Read the grid history through an output iterator
    template <typename OutputIter>
    OutputIter read_grid (OutputIter first) const
    {
        return std::copy(grid_.begin(), grid_.end(), first);
    }

    /// \brief Set a new evaluation object of type eval_type
    void set_eval (const eval_type &new_eval)
    {
        eval_ = new_eval;
    }

    /// Perform the evaluation for a given iteration and a Particle<T> object
    void eval (unsigned iter, const Particle<T> &particle)
    {
        VSMC_RUNTIME_ASSERT((bool(eval_)),
                ("CALL **Path::eval** WITH AN INVALID "
                 "EVALUATION FUNCTOR"));

        buffer_.resize(particle.size());
        weight_.resize(particle.size());
        particle.read_weight(&weight_[0]);

        index_.push_back(iter);
        width_.push_back(eval_(iter, particle, &buffer_[0]));
        grid_.push_back(grid_.size() ?
                grid_.back() + width_.back() : width_.back());
        integrand_.push_back(ddot_(
                    static_cast<typename ddot_type::size_type>(weight_.size()),
                    &weight_[0], 1, &buffer_[0], 1));
    }

    /// \brief Get the logarithm nomralizing constants ratio estimates
    double zconst () const
    {
        double sum = 0;
        for (unsigned i = 1; i != iter_size(); ++i)
            sum += 0.5 * width_[i] * (integrand_[i-1] + integrand_[i]);

        return sum;
    }

    /// \brief Clear all records of the index and integrations
    void clear ()
    {
        index_.clear();
        integrand_.clear();
        width_.clear();
        grid_.clear();
    }

    private :

    std::vector<double> buffer_;
    std::vector<double> weight_;
    eval_type eval_;
    std::vector<unsigned> index_;
    std::vector<double> integrand_;
    std::vector<double> width_;
    std::vector<double> grid_;
    ddot_type ddot_;
}; // class PathSampling

} // namespace vsmc

#endif // VSMC_CORE_PATH_HPP
