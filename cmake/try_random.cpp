#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#include <vSMC/internal/random.hpp>

int main ()
{
    r123::Engine<r123::Threefry4x64> eng(101);
    vSMC::internal::uniform_real_distribution<> runif(0, 1);

    int N = 10000000;
    int size = 20;
    double p = 0;
    std::cout.precision(15);
    while (p < 0.2 || p > 0.8)
        p = runif(eng);
    std::cout << "p: " << p << std::endl;

    int *y = new int[size + 1];
    for (int i = 0; i != size + 1; ++i)
        y[i] = 0;

    vSMC::internal::binomial_distribution<> rbinom(size, p);
    for (int i = 0; i != N; ++i)
        ++y[rbinom(eng)];

    for (int i = 0; i != size + 1; ++i)
        std::cout <<  y[i] / static_cast<double>(N) << std::endl;

    delete [] y;

    return 0;
}
