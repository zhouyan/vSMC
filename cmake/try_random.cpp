#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#include <vSMC/internal/random.hpp>

int main ()
{
    r123::Engine<r123::Threefry4x64> eng(101);
    vSMC::internal::binomial_distribution<> rbinom(100, 2);
    vSMC::internal::uniform_real_distribution<> runif(0, 1);

    int x = rbinom(eng);
    double y = runif(eng);

    return 0;
}
