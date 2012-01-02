#ifndef V_SMC_PARTICLE_HPP
#define V_SMC_PARTICLE_HPP

#include <cstddef>
#include <mkl_vml.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <vDist/utilities/buffer.hpp>

namespace vSMC {

/// The resampling scheme
enum ResampleScheme {MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC};

/// \brief Particle contains a set of particles and their weights.
///
/// The template parameter \b T is usually a container of the actual particel
/// types. It can also be other types other than a container. The important
/// thing is that it is not a single particle, but a set of them at a certain
/// iteration of the SMC algorithm. It has to have a contructor which accept
/// an integral type parameter, which is the number of particles, and the user
/// need to provide a function that tell Particle how to copy one particle to
/// another's position.
template <class T>
class Particle
{
    public :

    /// \brief The constructor of Particle. There is no default constructor.
    ///
    /// \param N The number of particles
    /// \param copy The pointer to a function which tell Particle how to copy
    /// a particle at position \b n1 to position \b n2.
    Particle (std::size_t N, void (*copy)(std::size_t, std::size_t, T &)) :
        particle_num(N), particle(N), weight(N), log_weight(N),
        replication(N), copy_particle(copy) {}

    /// \brief Get the number of particles
    ///
    /// \return The size of the particle set
    std::size_t size () const
    {
        return particle_num;
    }

    /// \brief Set the log(weights)
    ///
    /// \param [in] new_weight The pointer to the new log weights to be set
    void SetLogWeight (const double *new_weight)
    {
        cblas_dcopy(particle_num, new_weight, 1, log_weight, 1)
    }

    /// \brief Add to the log(weights)
    ///
    /// \param [in] inc_weight The pointer to the incremental log weights to
    /// be added
    void AddLogWeight (const double *inc_weight);
    {
        vdAdd(particle_num, log_weight, inc_weight, log_weight)
    }

    /// \brief Return the ESS
    ///
    /// \return The ESS calculated with current weights
    double ESS ()
    {
        require_weight();
        vdSqr(particle_num, weight, weight);
        return 1 / cblas_dasum(particle_num, weight, 1);
    }

    /// \brief Perform the resampling
    ///
    /// \param scheme See ResampleScheme
    /// \param rng The pointer to a GSL RNG object
    void Resample (ResampleScheme scheme, const gsl_rng *rng)
    {
        switch (scheme) {
            case MULTINOMIAL :
                resample_multinomial(rng);
                break;
            case RESIDUAL :
                resample_residual(rng);
                break;
            case STRATIFIED :
                resample_stratified(rng);
                break;
            case SYSTEMATIC :
                resample_systematic(rng);
                break;
            default :
                resample_residual(rng);
        }
    }

    private :

    typedef vDist::internal::Buffer<double> dBuffer;
    typedef vDist::internal::Buffer<std::size_t> uBuffer;

    const std::size_t particle_num;
    T particle;
    dBuffer weight;
    dBuffer log_weight;
    uBuffer replication;
    void (*copy_particle) (std::size_t, std::size_t, T &);

    void require_weight ()
    {
        vdExp(particle_num, log_weight, weight);
    }

    void resample_multinomial (const gsl_rng *rng)
    {
        require_weight();
        gsl_ran_multinomial(rng, particle_num, particle_num,
                weight, replication);

        resample_do();
    }

    void resample_residual (const gsl_rng *rng);
    {
        require_weight();
        cblas_dscal(particle_num, particle_num, weight, 1);
        vdFloor(particle_num, weight, weight);

        std::size_t size = particle_num;
        size -= cblas_dasum(particle_num, weight, 1);
        gsl_ran_multinomial(rng, particle_num, size, weight, replication);

        for (std::size_t i = 0; i != particle_num, ++i)
            replication[i] += weight[i];

        resample_do();
    }

    void resample_stratified (const gsl_rng *rng) {}
    void resample_systematic (const gsl_rng *rng) {}

    void resample_do ()
    {
        std::size_t from = 0;
        std::size_t time = 0;

        for (std::size_t i = 0; i != particle_num; ++i)
        {
            if (!replication[i]) { // replication[i] has zero child
                if (time == replication[from]) {
                    // all childs of replication[from] are already copied
                    time = 0;
                    ++from;
                    while (!replication[from])
                        ++from;
                }
                copy_particle(from, i, partile_set);
                ++time;
            }
        }
    }
}; // class Particle

} // namespace vSMC

#endif // V_SMC_PARTICLE_HPP
