# ============================================================================
#  vSMC/example/pf/pf.R
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

smp <- c("seq", "omp", "tbb")
exe <- character()
exe <- c(exe, paste("pf_smp", smp, sep = "_"))
exe <- c(exe, paste("pf_mpi", smp, sep = "_"))
exe <- c(exe, "pf_cl", "pf_cl_mpi")
res <- c(
    "Multinomial",
    "Residual",
    "Stratified",
    "Systematic",
    "ResidualStratified",
    "ResidualSystematic")
runs <- expand.grid(exe, res)
runs <- paste(runs$Var1, runs$Var2, sep = ".")

obs <- read.table("pf.data", header = FALSE)
dat.list <- data.frame(
    Position.X = obs[,1], Position.Y = obs[,2],
    Group = rep("Observation", dim(obs)[1]),
    Source = rep("Observation", dim(obs)[1]))
plt.list <- list()

pf <- function (suffix, run, rowcol, est)
{
    name <- paste(run, rowcol, ".", suffix, sep = "")
    dat <- data.frame(
        Position.X = c(obs[,1], est$pos.x),
        Position.Y = c(obs[,2], est$pos.y),
        Source = c(
            rep("Observation", dim(obs)[1]),
            rep("Estimates",   dim(est)[1])))
    plt <- qplot(x = Position.X, y = Position.Y, data = dat,
        group = Source, color = Source, linetype = Source, geom = "path")
    plt <- plt + ggtitle(name)
    .GlobalEnv$plt.list[[name]] <- plt

    dat <- data.frame(
        Position.X = est$pos.x, Position.Y = est$pos.y,
        Group = rep(name, dim(est)[1]),
        Source = rep("Estimates", dim(est)[1]))
    .GlobalEnv$dat.list <- rbind(.GlobalEnv$dat.list, dat)
}

for (rowcol in c(".row", ".col", "")) {
    for (run in runs) {
        pf.txt    <- paste(run, rowcol,        ".txt", sep = "")
        pf.r0.txt <- paste(run, rowcol, ".r0", ".txt", sep = "")
        pf.r1.txt <- paste(run, rowcol, ".r1", ".txt", sep = "")
        if (file.exists(pf.txt)) {
            pf("txt", run, rowcol,
                read.table(pf.txt, header = TRUE))
        } else if (file.exists(pf.r0.txt) && file.exists(pf.r1.txt)) {
            pf("txt", run, rowcol,
                read.table(pf.r0.txt, header = TRUE) +
                read.table(pf.r1.txt, header = TRUE))
        }

        pf.h5     <- paste(run, rowcol,        ".h5",  sep = "")
        pf.r0.h5  <- paste(run, rowcol, ".r0", ".h5",  sep = "")
        pf.r1.h5  <- paste(run, rowcol, ".r1", ".h5",  sep = "")
        if (file.exists(pf.h5)) {
            pf("h5", run, rowcol,
                as.data.frame(suppressWarnings(h5read(pf.h5, "/Sampler"))))
        } else if (file.exists(pf.r0.h5) && file.exists(pf.r1.h5)) {
            pf("h5", run, rowcol,
                as.data.frame(suppressWarnings(h5read(pf.r0.h5, "/Sampler"))) +
                as.data.frame(suppressWarnings(h5read(pf.r1.h5, "/Sampler"))))
        }
    }
}

plt <- qplot(x = Position.X, y = Position.Y, data = dat.list,
    group = Group, color = Source, linetype= Source, geom = "path")

pdf("pf.pdf", width = 14.4, height = 9)
print(plt)
for (plt in plt.list) print(plt)
garbage <- dev.off()
