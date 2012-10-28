#include <vsmc/helper/parallel_cl/query.hpp>

int main ()
{
    vsmc::QueryCL::print(std::cout);
    std::cout << vsmc::QueryCL() << std::endl;

    return 0;
}
