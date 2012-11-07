SET (VSMC_TYPE_TRAITS_TEST_SOURCE "
#include <cassert>
#include <vsmc/cxx11/type_traits.hpp>

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
    return func_impl<vsmc::cxx11::is_base_of<Base, T>::value>(value);
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
")

IF (NOT VSMC_TYPE_TRAITS_FOUND)
    UNSET (VSMC_TYPE_TRAITS_FOUND       CACHE)
    UNSET (VSMC_TYPE_TRAITS_STD_FOUND   CACHE)
    UNSET (VSMC_TYPE_TRAITS_BOOST_FOUND CACHE)
    INCLUDE (FindBoost)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS})
    SET (SAFE_CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES})
    SET (CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES}
        ${Boost_INCLUDE_DIRS})
    SET (CMAKE_REQUIRED_DEFINITIONS -DVSMC_HAS_CXX11LIB_TYPE_TRAITS=1)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_TYPE_TRAITS_TEST_SOURCE}"
        VSMC_TYPE_TRAITS_STD_FOUND)
    SET (CMAKE_REQUIRED_DEFINITIONS -DVSMC_HAS_CXX11LIB_TYPE_TRAITS=0)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_TYPE_TRAITS_TEST_SOURCE}"
        VSMC_TYPE_TRAITS_BOOST_FOUND)
    IF (VSMC_TYPE_TRAITS_STD_FOUND OR VSMC_TYPE_TRAITS_BOOST_FOUND)
        SET (VSMC_TYPE_TRAITS_FOUND TRUE)
    ENDIF (VSMC_TYPE_TRAITS_STD_FOUND OR VSMC_TYPE_TRAITS_BOOST_FOUND)
    SET (CMAKE_REQUIRED_DEFINITONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
ENDIF (NOT VSMC_TYPE_TRAITS_FOUND)
