#include <vsmc/rng/rng.hpp>

int main()
{
    const std::size_t n = 5;
    vsmc::Threefry4x32_64 rng(101);
    unsigned r[n];
    vsmc::uniform_bits_distribution(rng, n, r);
    for (std::size_t i = 0; i != n; ++i)
        std::cout << r[i] << '\t';
    std::cout << std::endl;

    return 0;
}
