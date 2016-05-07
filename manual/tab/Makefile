all : rng_distribution rng_engine_clang rng_engine_gcc rng_engine_intel
	./rng_distribution.pl
	./rng_engine.pl

rng_distribution :
	ninja -C ../../build/clang-Release rng_distribution
	ninja -C ../../build/clang-Release rng_distribution-check \
	    > rng_distribution.txt

rng_engine_clang :
	ninja -C ../../build/clang-Release rng_engine
	ninja -C ../../build/clang-Release rng_engine-check \
	    > rng_engine_clang.txt

rng_engine_gcc :
	ninja -C ../../build/gcc-Release rng_engine
	ninja -C ../../build/gcc-Release rng_engine-check \
	    > rng_engine_gcc.txt

rng_engine_intel :
	ninja -C ../../build/intel-Release rng_engine
	ninja -C ../../build/intel-Release rng_engine-check \
	    > rng_engine_intel.txt
