# Introduction

vSMC is a library that helps constructing SMC samplers. It is not very different from SMCTC in functionality. The main focus is performance.

The SMC algorithm admits straightaway parallelization. However, to take this advantage, one need to be able to access the whole particle sets in order to divide and conquer them. Therefore, the core modules of vSMC provide such accesses for exactly this purpose. In simple case, this mean the user will have to write a loop manipulate the whole particle set instead of a single one. In more complicated case, one can parallelize the manipulation, take advantage of vector instructions like SSE which can be found in most modern computers.

In addition, the library provides a set helper modules which assist users to write codes that manipulate single particle but run the program in parallel mode. The current supported backend is Intel Threading Building Block. Others may follow shortly.