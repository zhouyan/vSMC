.PHONY : all clean libvsmc

CC := clang
CFLAGS := $(CFLAGS) -std=c11 -O3 -DNDEBUG
CFLAGS := $(CFLAGS) -I ../../include
CFLAGS := $(CFLAGS) -L ../../build/clang-Release/lib
CFLAGS := $(CFLAGS) -Wl,-rpath -Wl,../../build/clang-Release/lib

CXX := clang++
CXXFLAGS := $(CXXFLAGS) -std=c++11 -O3 -DNDEBUG
CXXFLAGS := $(CXXFLAGS) -I ../../include
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_MKL
CXXFLAGS := $(CXXFLAGS) -DVSMC_HAS_TBB

LDFLAGS := $(LDFLAGS) -lmkl_rt -ltbb -ltbbmalloc -lm

all : pf_ocl pf_tbb pf_seq pf_tbb_c pf_seq_c program_option progress

libvsmc :
	ninja -C ../../build/clang-Release libvsmc_shared

pf.pdf : pf_tbb pf.R
	./pf_tbb
	Rscript pf.R

pf_ocl : pf_ocl.cpp
	$(CXX) $(CXXFLAGS) -DVSMC_HAS_OPENCL -o pf_ocl pf_ocl.cpp $(LDFLAGS) \
	    $(OPENCL_LIBRARY)

pf_tbb : pf_tbb.cpp
	$(CXX) $(CXXFLAGS) -o pf_tbb pf_tbb.cpp $(LDFLAGS)

pf_seq : pf_seq.cpp
	$(CXX) $(CXXFLAGS) -o pf_seq pf_seq.cpp $(LDFLAGS)

pf_tbb_c : pf_tbb.c libvsmc
	$(CC) $(CFLAGS) -o pf_tbb_c pf_tbb.c -lvsmc $(LDFLAGS)

pf_seq_c : pf_seq.c libvsmc
	$(CC) $(CFLAGS) -o pf_seq_c pf_seq.c -lvsmc $(LDFLAGS)

pf_bench : pf_tbb pf_seq pf_tbb_c pf_seq_c
	./pf_tbb
	./pf_seq
	./pf_tbb_c
	./pf_seq_c

program_option : program_option.cpp
	$(CXX) $(CXXFLAGS) -o program_option program_option.cpp $(LDFLAGS)

progress : progress.cpp
	$(CXX) $(CXXFLAGS) -o progress progress.cpp $(LDFLAGS)

clean:
	rm -f pf_ocl
	rm -f pf_seq pf_tbb
	rm -f pf_seq_c pf_tbb_c
	rm -f pf.out pf.rout pf.pdf
	rm -f program_option
	rm -f progress
