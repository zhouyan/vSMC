#include <cassert>

#ifndef __BLOCKS__
#error No __BLOCKS__
#endif

int main ()
{
    int multiplier = 7;
    int (^myBlock)(int) = ^(int num) {
        return num * multiplier;
    };

    assert(myBlock(3) == 21);

    return 0;
}
