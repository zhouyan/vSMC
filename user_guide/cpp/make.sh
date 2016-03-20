#!/bin/bash

source ~/Documents/GitHub/Script/OS/`uname`

make -j$ncpu -f Makefile.`uname` $1
