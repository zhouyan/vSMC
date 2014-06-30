//============================================================================
// cmake/FindOpenCL.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <cl.hpp>
#include <vector>
#include <cassert>

int main ()
{
    std::vector<cl::Platform> plat;
    cl::Platform::get(&plat);
    std::vector<cl::Device> dev;
    plat[0].getDevices(CL_DEVICE_TYPE_ALL, &dev);
    assert(dev.size() != 0);

    return 0;
}
