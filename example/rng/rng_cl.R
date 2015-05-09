# ============================================================================
#  vSMC/example/rng/rng_cl.R
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013-2015, Yan Zhou
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# ============================================================================

suppressPackageStartupMessages(library(ggplot2))
suppressPackageStartupMessages(library(rhdf5))

theme_set(theme_bw())

if (file.exists("rng_cl.h5")) {
    result <- h5read("rng_cl.h5", "result")
} else {
    result <- read.table("rng_cl.txt", header = FALSE)
}

N <- dim(result)[1]
Implementation <- rep(c("C++", "C", "OpenCL", "R"), each = N)

Value <- c(as.vector(result[,1:3]), rnorm(N, 0, 1))
Normal01 <- data.frame(Value = Value, Implementation = Implementation)

Value <- c(as.vector(result[,4:6]), rgamma(N, 0.01, 1))
GammaK1_0.01 <- data.frame(Value = Value, Implementation = Implementation)

Value <- c(as.vector(result[,7:9]), rgamma(N, 0.1, 1))
GammaK1_0.1 <- data.frame(Value = Value, Implementation = Implementation)

Value <- c(as.vector(result[,10:12]), rgamma(N, 1, 1))
GammaK1_1 <- data.frame(Value = Value, Implementation = Implementation)

Value <- c(as.vector(result[,13:15]), rgamma(N, 10, 1))
GammaK1_10 <- data.frame(Value = Value, Implementation = Implementation)

Value <- c(as.vector(result[,16:18]), rgamma(N, 100, 1))
GammaK1_100 <- data.frame(Value = Value, Implementation = Implementation)

Plt <- qplot(Value, group = Implementation, color = Implementation,
    geom = "density")

pdf("rng_cl.pdf", width = 14.4, height = 9)
print(Plt %+% Normal01 + ggtitle("Normal(0, 1)"))
print(Plt %+% GammaK1_0.01 + ggtitle("Gamma(0.01, 1)"))
print(Plt %+% GammaK1_0.1  + ggtitle("Gamma(0.1, 1)"))
print(Plt %+% GammaK1_1    + ggtitle("Gamma(1, 1)"))
print(Plt %+% GammaK1_10   + ggtitle("Gamma(10, 1)"))
print(Plt %+% GammaK1_100  + ggtitle("Gamma(100, 1)"))
garbarg <- dev.off();
