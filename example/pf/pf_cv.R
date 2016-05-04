# ============================================================================
#  vSMC/example/pf/pf_cv.R
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

smp <- c("OMP", "SEQ", "TBB")
exe <- paste("pf_cv", smp, sep = ".")
res <- c(
    "Multinomial",
    "Stratified",
    "Systematic",
    "Residual",
    "ResidualStratified",
    "ResidualSystematic")
rc <- c("RowMajor", "ColMajor")
rs <- c("RNGSetVector", "RNGSetTBB")
runs <- expand.grid(exe, res, rc, rs)
runs <- paste(runs$Var1, runs$Var2, runs$Var3, runs$Var4, sep = ".")

pf_cv_est <- function() {
    src.level <- c("Estimates", "Observation")
    obs <- read.table("pf_cv.data", header = FALSE)
    dat <- data.frame(
        Position.X = obs[,1], Position.Y = obs[,2],
        Group = rep("Observation", dim(obs)[1]),
        Source = rep("Observation", dim(obs)[1]))
    plt.list <- list()

    pf_cv <- function (filename, est)
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

    for (run in runs) {
        pf_cv.txt <- paste0(run, ".txt")
        if (file.exists(pf_cv.txt)) {
            tmp <- pf_cv(pf_cv.txt, read.table(pf_cv.txt, header = TRUE))
            plt.list[[pf_cv.txt]] <- tmp$plt
            dat <- rbind(dat, tmp$dat)
        }

        pf_cv.h5 <- paste0(run, ".h5")
        if (file.exists(pf_cv.h5)) {
            pf_cv.s <- paste(pf_cv.h5, "(Sampler)")
            tmp <- pf_cv(pf_cv.s, as.data.frame(h5read(pf_cv.h5, "Sampler")))
            plt.list[[pf_cv.s]] <- tmp$plt
            dat <- rbind(dat, tmp$dat)

            pf_cv.m <- paste(pf_cv.h5, "(Monitor)")
            tmp <- pf_cv(pf_cv.m, as.data.frame(h5read(pf_cv.h5, "Monitor")))
            plt.list[[pf_cv.m]] <- tmp$plt
            dat <- rbind(dat, tmp$dat)

            pf_cv.p <- paste(pf_cv.h5, "(Particle)")
            particle <- h5read(pf_cv.h5, "Particle")
            pos.x <- numeric()
            pos.y <- numeric()
            for (i in 0:(length(particle) - 1)) {
                name <- paste0("Iter.", i)
                w <- particle[[name]]$Weight
                s <- particle[[name]]$Value
                if (dim(s)[1] > dim(s)[2]) {
                    pos.x <- c(pos.x, sum(w * s[,1]))
                    pos.y <- c(pos.y, sum(w * s[,2]))
                } else {
                    pos.x <- c(pos.x, sum(w * s[1,]))
                    pos.y <- c(pos.y, sum(w * s[2,]))
                }
            }
            tmp <- pf_cv(pf_cv.p, as.data.frame(cbind(pos.x, pos.y)))
            plt.list[[pf_cv.p]] <- tmp$plt
            dat <- rbind(dat, tmp$dat)
        }
    }

    dat$Source <- factor(dat$Source, ordered = TRUE, levels = src.level)
    plt <- qplot(x = Position.X, y = Position.Y, data = dat,
        group = Group, color = Source, linetype= Source, geom = "path")
    plt <- plt + ggtitle("pf_cv")

    list(plot = plt, plot.list = plt.list)
}

pdf("pf_cv.pdf", width = 14.4, height = 9)
est <- pf_cv_est()
print(est$plot)
for (p in est$plot.list) print(p)
garbage <- dev.off()
