#include <chrono>
#include <iostream>

int main ()
{
    std::chrono::time_point<std::chrono::system_clock> bt =
        std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> et =
        std::chrono::system_clock::now();
    std::cout <<
        std::chrono::duration_cast<std::chrono::microseconds>(et - bt)
        .count() << std::endl;

    return 0;
}