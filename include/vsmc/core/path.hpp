#ifndef VSMC_CORE_PATH_HPP
#define VSMC_CORE_PATH_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Monitor for path sampling
/// \ingroup Core
///
/// \tparam T Particle<T>::value_type
template <typename T>
class Path
{
    public :

    /// The type of the particle values
    typedef T value_type;

    /// The type of path sampling evaluation functor
    typedef cxx11::function<double (
            unsigned, const Particle<T> &, double *)> eval_type;

    /// \brief Construct a Path with an evaluation function
    ///
    /// \param eval The functor used to compute the integrands
    explicit Path (const eval_type &eval = VSMC_NULLPTR) : eval_(eval) {}

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

    /// Size of records
    unsigned iter_size () const
    {
        return static_cast<unsigned>(index_.size());
    }

    /// \brief Test if the monitor is valid
    ///
    /// \note This operator will be \c explicit if the C++11 feature is enabled
#if VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
    explicit
#endif
        operator bool () const
    {
        return bool(eval_);
    }

    unsigned index (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::index** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return index_[iter];
    }

    double integrand (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::integrand** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return integrand_[iter];
    }

    double width (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::width** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return width_[iter];
    }

    double grid (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::grid** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return grid_[iter];
    }

    /// \brief Read only access to iteration index
    ///
    /// \param first The beginning of the destination range
    ///
    /// \return Output iterator to the element in the destination range, one
    /// past the last element copied
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        return std::copy(index_.begin(), index_.end(), first);
    }

    /// \brief Read only access to iteration integrand
    ///
    /// \param first The beginning of the destination range
    ///
    /// \return Output iterator to the element in the destination range, one
    /// past the last element copied
    template <typename OutputIter>
    OutputIter read_integrand (OutputIter first) const
    {
        return std::copy(integrand_.begin(), integrand_.end(), first);
    }

    /// \brief Read only access to iteration width
    ///
    /// \param first The beginning of the destination range
    ///
    /// \return Output iterator to the element in the destination range, one
    /// past the last element copied
    template <typename OutputIter>
    OutputIter read_width (OutputIter first) const
    {
        return std::copy(width_.begin(), width_.end(), first);
    }

    /// \brief Read only access to iteration grid
    ///
    /// \param first The beginning of the destination range
    ///
    /// \return Output iterator to the element in the destination range, one
    /// past the last element copied
    template <typename OutputIter>
    OutputIter read_grid (OutputIter first) const
    {
        return std::copy(grid_.begin(), grid_.end(), first);
    }

    /// Set the evaluation functor
    void set_eval (const eval_type &new_eval)
    {
        eval_ = new_eval;
    }

    /// \brief Evaluate the integration
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    void eval (unsigned iter, const Particle<T> &particle)
    {
        VSMC_RUNTIME_ASSERT((bool(eval_)),
                ("CALL **Path::eval** WITH AN INVALID "
                 "EVALUATION FUNCTOR"));

        buffer_.resize(particle.size());
        weight_.resize(particle.size());
        double w = eval_(iter, particle, &buffer_[0]);
        particle.read_weight(weight_.begin());
        double p = 0;
        for (std::vector<double>::size_type i = 0; i != weight_.size(); ++i)
            p += weight_[i] * buffer_[i];

        index_.push_back(iter);
        integrand_.push_back(p);
        width_.push_back(w);
        grid_.push_back(grid_.size() ?
                grid_.back() + width_.back() : width_.back());
    }

    /// Path sampling estimate of normalizing constant
    double zconst () const
    {
        double sum = 0;
        for (unsigned i = 1; i != iter_size(); ++i)
            sum += 0.5 * width_[i] * (integrand_[i-1] + integrand_[i]);

        return sum;
    }

    /// \brief Clear all recorded data
    ///
    /// \note The evaluation functor is not reset
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
}; // class PathSampling

} // namespace vsmc

#endif // VSMC_CORE_PATH_HPP
