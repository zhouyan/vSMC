#include <cassert>

int main ()
{
    unsigned eax = 1;
    unsigned ebx = 0;
    __asm__ ("movl %%eax, %%ebx;" : "=b" (ebx) : "a" (eax));
    assert(ebx == eax);

    return 0;
}
