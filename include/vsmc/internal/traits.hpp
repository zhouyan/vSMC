#ifndef VSMC_INTERNAL_TRAITS_HPP
#define VSMC_INTERNAL_TRAITS_HPP

namespace vsmc {

class SizeTypeTrait
{
    public :

    /// The type of the size of the particle set
    typedef VSMC_INDEX_TYPE size_type;
};

class SamplerTrait :            public SizeTypeTrait {};
class ParticleTrait :           public SizeTypeTrait {};
class MonitorTrait :            public SizeTypeTrait {};
class PathTrait :               public SizeTypeTrait {};

class StateBaseTrait :          public SizeTypeTrait {};
class SingleParticleTrait:      public SizeTypeTrait {};
class ConstSingleParticleTrait: public SizeTypeTrait {};

class StateSeqTrait :           public SizeTypeTrait {};
class InitializeSeqTrait :      public SizeTypeTrait {};
class MoveSeqTrait :            public SizeTypeTrait {};
class MonitorSeqTrait :         public SizeTypeTrait {};
class PathSeqTrait :            public SizeTypeTrait {};

class StateTBBTrait :           public SizeTypeTrait {};
class InitializeTBBTrait :      public SizeTypeTrait {};
class MoveTBBTrait :            public SizeTypeTrait {};
class MonitorTBBTrait :         public SizeTypeTrait {};
class PathTBBTrait :            public SizeTypeTrait {};

class StateCLTrait      {};
class InitializeCLTrait {};
class MoveCLTrait       {};
class MonitorCLTrait    {};
class PathCLTrait       {};

} // namespace vsmc

#endif // VSMC_INTERNAL_TRAITS_HPP
