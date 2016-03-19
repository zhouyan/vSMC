CXXFLAGS := $(CXXFLAGS) -std=c++11 -O3
CXXFLAGS := $(CXXFLAGS) -I ../../include
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_MKL
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_TBB
CXXFLAGS_CL := $(CXXFLAGS) -DVSMC_HAS_OPENCL

LDFLAGS := -lmkl_rt -ltbb -ltbbmalloc
LDFLAGS_CL := $(LDFLAGS) -lOpenCL

all : pf_ocl pf_tbb pf_seq program_option progress

pf_ocl : pf_ocl.cpp
	$(CXX) $(CXXFLAGS_CL) -o pf_ocl pf_ocl.cpp $(LDFLAGS_CL)

pf_tbb : pf_tbb.cpp
	$(CXX) $(CXXFLAGS) -o pf_tbb pf_tbb.cpp $(LDFLAGS)

pf_seq : pf_seq.cpp
	$(CXX) $(CXXFLAGS) -o pf_seq pf_seq.cpp $(LDFLAGS)

program_option : program_option.cpp
	$(CXX) $(CXXFLAGS) -o program_option program_option.cpp $(LDFLAGS)

progress : program_option.cpp
	$(CXX) $(CXXFLAGS) -o progress progress.cpp $(LDFLAGS)

clean:
	rm -f pf_ocl pf_seq pf_tbb
	rm -f pf.out pf.rout pf.pdf
	rm -f program_option
	rm -f progress
