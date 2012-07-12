#include <vsmc/timer/chrono_timer.hpp>

int main ()
{
    vsmc::ChronoTimer timer;
    timer.start();
    timer.stop();
    double x = timer.duration();

    return 0;
}
