#include <cassert>

template <typename F>
void invoke (const F &f) {f();}

int main ()
{
    int i = 0;
    invoke([&]{i = 1;});
    assert(i == 1);

    return 0;
}
