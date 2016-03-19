CXX := clang++
CXXFLAGS := $(CXXFLAGS) -std=c++11 -O3
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_MKL
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_TBB
CXXFLAGS := $(CXXFLAGS) -I ~/Documents/GitHub/vSMC/include
LDFLAGS := -lmkl_rt -ltbb -ltbbmalloc

all : pf_seq pf_tbb progress program_option

pf_seq : pf_seq.cpp
	$(CXX) $(CXXFLAGS) -o pf_seq pf_seq.cpp $(LDFLAGS)

pf_tbb : pf_tbb.cpp
	$(CXX) $(CXXFLAGS) -o pf_tbb pf_tbb.cpp $(LDFLAGS)

program_option : program_option.cpp
	$(CXX) $(CXXFLAGS) -o program_option program_option.cpp $(LDFLAGS)

progress : program_option.cpp
	$(CXX) $(CXXFLAGS) -o progress progress.cpp $(LDFLAGS)

clean:
	rm -f pf_seq pf_tbb pf.out pf.rout pf.pdf
	rm -f program_option
	rm -f progress
