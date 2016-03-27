# ============================================================================
#  vSMC/example/pf/pf.R
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013-2016, Yan Zhou
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
exe <- paste("pf", smp, sep = "_")
res <- c(
    "Multinomial",
    "Stratified",
    "Systematic",
    "Residual",
    "ResidualStratified",
    "ResidualSystematic")
runs <- expand.grid(exe, res)
runs <- paste(runs$Var1, runs$Var2, sep = ".")

src.level <- c("Estimates", "Observation")
obs <- read.table("pf.data", header = FALSE)
dat <- data.frame(
    Position.X = obs[,1], Position.Y = obs[,2],
    Group = rep("Observation", dim(obs)[1]),
    Source = rep("Observation", dim(obs)[1]))
plt.list <- list()

pf <- function (filename, est)
{
    dat <- data.frame(
        Position.X = c(obs[,1], est$pos.x),
        Position.Y = c(obs[,2], est$pos.y),
        Source = c(
            rep("Observation", dim(obs)[1]),
            rep("Estimates",   dim(est)[1])))
    dat$Source <- factor(dat$Source, ordered = TRUE, levels = src.level)
    plt <- qplot(x = Position.X, y = Position.Y, data = dat,
        group = Source, color = Source, linetype = Source, geom = "path")
    plt <- plt + ggtitle(filename)

    dat <- data.frame(
        Position.X = est$pos.x, Position.Y = est$pos.y,
        Group = rep(filename, dim(est)[1]),
        Source = rep("Estimates", dim(est)[1]))

    list(plt = plt, dat = dat)
}

for (rowcol in c(".row", ".col")) {
    for (run in runs) {
        pf.txt <- paste0(run, rowcol, ".txt")
        if (file.exists(pf.txt)) {
            tmp <- pf(pf.txt, read.table(pf.txt, header = TRUE))
            plt.list[[pf.txt]] <- tmp$plt
            dat <- rbind(dat, tmp$dat)
        }
        pf.h5 <- paste0(run, rowcol, ".h5")
        if (file.exists(pf.h5)) {
            tmp <- pf(pf.h5, as.data.frame(h5read(pf.h5, "/Sampler")))
            plt.list[[pf.h5]] <- tmp$plt
            dat <- rbind(dat, tmp$dat)
        }
    }
}

dat$Source <- factor(dat$Source, ordered = TRUE, levels = src.level)
plt <- qplot(x = Position.X, y = Position.Y, data = dat,
    group = Group, color = Source, linetype= Source, geom = "path")
plt <- plt + ggtitle("pf")

pdf("pf.pdf", width = 14.4, height = 9)
print(plt)
for (plt in plt.list) print(plt)
garbage <- dev.off()
