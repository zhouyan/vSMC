#ifndef VSMC_HELPER_BAYESIAN_ALPHA_HPP
#define VSMC_HELPER_BAYESIAN_ALPHA_HPP

namespace vsmc {

template <typename T>
class NullAlpha
{
    public :

    typedef unsigned value_type;

    void operator () (unsigned iter, Particle<T> &) {}
}; // class NullAlpha

template <typename T>
class LinearAlpha
{
    public :

    typedef unsigned value_type;

    explicit LinearAlpha (unsigned iter_num) : iter_num_(iter_num) {}

    void operator () (unsigned iter, Particle<T> &particle)
    {
        particle.value().alpha(static_cast<double>(iter) / iter_num_);
    }

    private :

    unsigned iter_num_;
}; // class LinearAlpha

template <typename T>
class PriorAlpha
{
    public :

    typedef std::pair<unsigned, double> value_type;

    explicit PriorAlpha (const value_type &config) :
        iter_num_(config.first), power_(config.second) {}

    void operator () (unsigned iter, Particle<T> &particle)
    {
        using std::pow;

        double b = static_cast<double>(iter) / iter_num_;
        particle.value().alpha(pow(b, power_));
    }

    private :

    unsigned iter_num_;
    double power_;
}; // class PriorAlpha

template <typename T>
class PosteriorAlpha
{
    public :

    typedef std::pair<unsigned, double> value_type;

    explicit PosteriorAlpha (const value_type &config) :
        iter_num_(config.first), power_(config.second) {}

    void operator () (unsigned iter, Particle<T> &particle)
    {
        using std::pow;

        double b = 1 - static_cast<double>(iter) / iter_num_;
        particle.value().alpha(1 - pow(b, power_));
    }

    private :

    unsigned iter_num_;
    double power_;
}; // class PosteriorAlpha

} // namespace vsmc

#endif // VSMC_HELPER_BAYESIAN_ALPHA_HPP
