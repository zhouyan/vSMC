# ============================================================================
#  vSMC/example/pet/pet_conv.R
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

# Use multicore for pre-2.14 R, 2.14 and after use parallel
Version <- as.numeric(version$major) + as.numeric(version$minor) / 10
if (Version >= 2.14) library(parallel) else library(multicore)
library(stats)

load("pet_conv.RData")

InputFun <- splinefun(Input$time, Input$CP)

BasisFun <- function(x, uptime, theta)
{exp(-theta * (uptime - x)) * InputFun(x)}

Convolution <- function(theta, t)
{
    integrate(BasisFun, lower=0, upper=t, uptime=t, theta=theta,
              subdivisions=1e8, stop.on.error=FALSE)$value
}

ComputeConv <- function(theta_a, theta_b, theta_step)
{
    Theta <- seq(theta_a, theta_b, theta_step)
    m <- length(Theta)
    n <- length(TimeFrame)

    ConvList <- list()

    for (i in 1:n) {
        cat("Time frame: ", i, "\n")
        tf <- TimeFrame[i]
        ConvList[[i]] <- mclapply(Theta, Convolution, t = tf)
    }

    ConvMat <- matrix(rep(0, m * n), ncol = n)
    for (j in 1:n) {
        for (i in 1:m) {
            ConvMat[i,j] <- ConvList[[j]][[i]]
        }
    }

    list(ConvMat=ConvMat, Theta=Theta)
}

ConvMat <- ComputeConv(0, 1 + 1e-5, 1e-5)
write.table(ConvMat, "pet_conv.data", row.names = FALSE, col.names = FALSE)
