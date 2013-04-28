#include <cilk/cilk.h>
#include <cilk/reducer_opadd.h>

int main ()
{
    unsigned N = 1000;
    unsigned C = (N * (N - 1)) / 2;
    cilk::reducer_opadd<unsigned> T;
    cilk_for(unsigned i = 0; i != N; ++i) T += i;

    return T.get_value() != C;
}
