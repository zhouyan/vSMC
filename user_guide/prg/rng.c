#include <vsmc/rng/rng.h>
#include <stdio.h>

int main()
{
    const size_t n = 5;
    vsmc_rng rng = vsmc_rng_new(101, vSMCThreefry4x32_64);
    unsigned r[n];
    vsmc_rng_rand(rng, n, r);
    vsmc_rng_delete(&rng);

    for (size_t i = 0; i < n; i++)
        printf("%u\t", r[i]);
    printf("\n");

    return 0;
}