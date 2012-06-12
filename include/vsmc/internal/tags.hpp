#ifndef VSMC_INTERNAL_TAGS_HPP
#define VSMC_INTERNAL_TAGS_HPP

namespace vsmc { namespace internal {

class ParallelTag  {};

class StateSeqTag      {};
class InitializeSeqTag {};
class MoveSeqTag       {};
class MonitorSeqTag    {};
class PathSeqTag       {};

class StateTBBTag :      public ParallelTag {};
class InitializeTBBTag : public ParallelTag {};
class MoveTBBTag :       public ParallelTag {};
class MonitorTBBTag :    public ParallelTag {};
class PathTBBTag :       public ParallelTag {};

class StateCLTag :       public ParallelTag {};
class InitializeCLTag :  public ParallelTag {};
class MoveCLTag :        public ParallelTag {};
class MonitorCLTag :     public ParallelTag {};
class PathCLTag :        public ParallelTag {};

} } // namespace vsmc::internal

#endif // VSMC_INTERNAL_TAGS_HPP
