#include <cassert>
#include <vSMC/internal/type_traits.hpp>

class Base
{
    public :

    virtual int val ()
    {
        return 1;
    }
};
class Derived : public Base
{
    public :

    int val ()
    {
        return 2;
    }
};

template <bool if_base>
int func_impl (void *value)
{
    return 0;
}

template<>
int func_impl<true>(void *value)
{
    return reinterpret_cast<Base *>(value)->val();
}

template <typename T>
int func (T *value)
{
    return func_impl<vSMC::internal::is_base_of<Base, T>::value>(value);
}

int main ()
{
    Base base;
    Derived derived;
    int a = 0;

    assert(func(&a) == 0);
    assert(func(&base) == 1);
    assert(func(&derived) == 2);

    return 0;
}
