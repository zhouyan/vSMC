.PHONY : all clean libvsmc

CC := clang
CFLAGS := $(CFLAGS) -O3 -DNDEBUG
CFLAGS := $(CFLAGS) -I ../../include
CFLAGS := $(CFLAGS) -L ../../build/clang-Release/lib
CFLAGS := $(CFLAGS) -Wl,-rpath -Wl,../../build/clang-Release/lib

CXX := clang++
CXXFLAGS := $(CXXFLAGS) -std=c++14 -O3 -DNDEBUG
CXXFLAGS := $(CXXFLAGS) -I ../../include
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_MKL
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_TBB

LDFLAGS := $(LDFLAGS) -lmkl_rt -ltbb -ltbbmalloc -lm

all : pf_seq pf_tbb pf_ocl program_option progress

libvsmc :
	ninja -C ../../build/clang-Release libvsmc_shared

gmm : gmm.cpp
	$(CXX) $(CXXFLAGS) -o gmm gmm.cpp $(LDFLAGS)

pf.pdf : pf_tbb pf.R
	./pf_tbb
	Rscript pf.R

pf_seq : pf_seq.c libvsmc
	$(CC) $(CFLAGS) -o pf_seq pf_seq.c -lvsmc $(LDFLAGS)

pf_tbb : pf_tbb.c libvsmc
	$(CC) $(CFLAGS) -o pf_tbb pf_tbb.c -lvsmc $(LDFLAGS)

pf_ocl : pf_ocl.cpp
	$(CXX) $(CXXFLAGS) -DVSMC_HAS_OPENCL -o pf_ocl pf_ocl.cpp $(LDFLAGS) \
	    $(OPENCL_LIBRARY)

program_option : program_option.cpp
	$(CXX) $(CXXFLAGS) -o program_option program_option.cpp $(LDFLAGS)

progress : progress.cpp
	$(CXX) $(CXXFLAGS) -o progress progress.cpp $(LDFLAGS)

clean:
	rm -f pf_seq pf_tbb pf_ocl pf.out pf.rout pf.pdf
	rm -f program_option
	rm -f progress
