#ifndef V_SMC_INTERNAL_TRAITS_HPP
#define V_SMC_INTERNAL_TRAITS_HPP

namespace vSMC {

class StateBaseTrait
{
    public :

    virtual ~StateBaseTrait () {}
};

class StateSeqTrait
{
    public :

    virtual ~StateSeqTrait () {}
};

class InitializeSeqTrait
{
    public :

    virtual ~InitializeSeqTrait () {}
};

class MoveSeqTrait
{
    public :

    virtual ~MoveSeqTrait () {}
};

class MonitorSeqTrait
{
    public :

    virtual ~MonitorSeqTrait () {}
};

class PathSeqTrait
{
    public :

    virtual ~PathSeqTrait () {}
};

class StateTBBTrait
{
    public :

    virtual ~StateTBBTrait () {}

    virtual void pre_resampling () = 0;
    virtual void post_resampling () = 0;
};

class InitializeTBBTrait
{
    public :

    virtual ~InitializeTBBTrait () {}
};

class MoveTBBTrait
{
    public :

    virtual ~MoveTBBTrait () {}
};

class MonitorTBBTrait
{
    public :

    virtual ~MonitorTBBTrait () {}
};

class PathTBBTrait
{
    public :

    virtual ~PathTBBTrait () {}
};

class StateCLTrait
{
    public :

    virtual ~StateCLTrait () {}

    virtual void pre_resampling () = 0;
    virtual void post_resampling () = 0;
};

class InitializeCLTrait
{
    public :

    virtual ~InitializeCLTrait () {}
};

class MoveCLTrait
{
    public :

    virtual ~MoveCLTrait () {}
};

class MonitorCLTrait
{
    public :

    virtual ~MonitorCLTrait () {}
};

class PathCLTrait
{
    public :

    virtual ~PathCLTrait () {}
};

} // namespace vSMC

#endif // V_SMC_INTERNAL_TRAITS_HPP
