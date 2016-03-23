CC := clang
CFLAGS := $(CFLAGS) -std=c89 -O3
CFLAGS := $(CFLAGS) -I ../../include
CFLAGS := $(CFLAGS) -L ../../build/$(CC)-Release/lib
CFLAGS := $(CFLAGS) -rpath ../../build/$(CC)-Release/lib

CXX := clang++
CXXFLAGS := $(CXXFLAGS) -std=c++11 -O3
CXXFLAGS := $(CXXFLAGS) -I ../../include
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_MKL
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_TBB
LDFLAGS := -lmkl_rt -ltbb -ltbbmalloc

all : pf_ocl pf_tbb pf_seq program_option program_progress

pf_ocl : pf_ocl.cpp
	$(CXX) $(CXXFLAGS) -DVSMC_HAS_OPENCL -o pf_ocl pf_ocl.cpp $(LDFLAGS) $(OPENCL_LIBRARY)

pf_tbb : pf_tbb.cpp
	$(CXX) $(CXXFLAGS) -o pf_tbb pf_tbb.cpp $(LDFLAGS)

pf_seq : pf_seq.cpp
	$(CXX) $(CXXFLAGS) -o pf_seq pf_seq.cpp $(LDFLAGS)

pf_tbb_c : pf_tbb.c
	ninja -C ../../build/$(CC)-Release lib
	$(CC) $(CFLAGS) -o pf_tbb_c pf_tbb.c $(LDFLAGS) -lvsmc

pf_seq_c : pf_seq.c
	ninja -C ../../build/$(CC)-Release lib
	$(CC) $(CFLAGS) -o pf_seq_c pf_seq.c $(LDFLAGS) -lvsmc

program_option : program_option.cpp
	$(CXX) $(CXXFLAGS) -o program_option program_option.cpp $(LDFLAGS)

program_process : program_process.cpp
	$(CXX) $(CXXFLAGS) -o program_process program_process.cpp $(LDFLAGS)

clean:
	rm -f pf_ocl pf_seq pf_tbb pf_seq_c pf_tbb_c
	rm -f pf.out pf.rout pf.pdf
	rm -f program_option
	rm -f program_progress
