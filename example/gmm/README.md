# Gaussian mixture model

This example implement algorithms of Bayesian model comparison for the Gaussian
mixture model as found in:

Del Moral, P., Doucet, A., & Jasra, A. (2006). Sequential Monte Carlo samplers.
Journal of the Royal Statistical Society: Series B (Statistical Methodology),
68(3), 411-436.

## Algorithms

- `gmm_pmcmc`: Population Markov chain Monte Carlo
- `gmm_rjmcmc`: Reversible jump Markov chain Monte Carlo
- `gmm_rjsmc`: Sequential Monte Carlo operating on the same space as RJMCMC
- `gmm_smc`: Sequential Monte Carlo operating on the space of the model
  posterior
- `gmm_smc_pair`: Sequential Monte Carlo operating on joint space of two models
