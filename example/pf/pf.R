# ============================================================================
#  vSMC/vSMCExample/pf/pf.R
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013,2014, Yan Zhou
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

theme_set(theme_bw())

execnames <- c("seq", "cilk", "gcd", "omp", "std", "tbb")
basenames <- character()
basenames <- c(basenames, paste("pf_matrix", execnames, sep = "_"))
basenames <- c(basenames, paste("pf_tuple", execnames, sep = "_"))
basenames <- c(basenames, paste("pf_matrix_mpi", execnames, sep = "_"))
basenames <- c(basenames, paste("pf_tuple_mpi", execnames, sep = "_"))
basenames <- c(basenames, "pf_cl")
resenames <- c(
    "Multinomial",
    "Residual",
    "Stratified",
    "Systematic",
    "ResidualStratified",
    "ResidualSystematic")
filenames <- expand.grid(basenames, resenames)
filenames <- paste(filenames$Var1, filenames$Var2, sep = ".")

obs <- read.table("pf.data", header = FALSE)
dat.list <- data.frame(
    Position.X = obs[,1], Position.Y = obs[,2],
    Group = rep("Observation", dim(obs)[1]),
    Source = rep("Observation", dim(obs)[1]))
plt.list <- list()
for (rowcol in c(".row", ".col", "")) {
    for (filename in filenames) {
        read.xy <- FALSE
        save_file.txt  <- paste(filename, rowcol,        ".txt", sep = "")
        save_file0.txt <- paste(filename, rowcol, ".r0", ".txt", sep = "")
        save_file1.txt <- paste(filename, rowcol, ".r1", ".txt", sep = "")
        save_file.hdf5  <- paste(filename, rowcol,        ".hdf5", sep = "")
        save_file0.hdf5 <- paste(filename, rowcol, ".r0", ".hdf5", sep = "")
        save_file1.hdf5 <- paste(filename, rowcol, ".r1", ".hdf5", sep = "")
        if (file.exists(save_file.hdf5)) {
            suppressPackageStartupMessages(library(rhdf5))
            xy <- as.data.frame(h5read(save_file.hdf5, "/Sampler"))
            read.xy <- TRUE
        } else if (file.exists(save_file.txt)) {
            xy <- read.table(save_file.txt, header = TRUE)
            read.xy <- TRUE
        } else if (file.exists(save_file0.hdf5) && file.exists(save_file1.hdf5)) {
            xy0 <- as.data.frame(h5read(save_file0.hdf5, "/Sampler"))
            xy1 <- as.data.frame(h5read(save_file1.hdf5, "/Sampler"))
            xy <- xy0 + xy1
            read.xy <- TRUE
        } else if (file.exists(save_file0.txt) && file.exists(save_file1.txt)) {
            xy0 <- read.table(save_file0, header = TRUE)
            xy1 <- read.table(save_file1, header = TRUE)
            xy <- xy0 + xy1
            read.xy <- TRUE
        }
        if (read.xy) {
            name <- paste(filename, rowcol, sep = "")
            dat <- data.frame(
                Position.X = c(obs[,1], xy$pos.x),
                Position.Y = c(obs[,2], xy$pos.y),
                Source = c(
                    rep("Observation", dim(obs)[1]),
                    rep("Estimates",   dim(xy)[1])))
            plt <- qplot(x = Position.X, y = Position.Y, data = dat,
                group = Source, color = Source, linetype = Source, geom = "path")
            plt <- plt + ggtitle(name)
            plt.list[[name]] <- plt
            dat <- data.frame(
                Position.X = xy$pos.x, Position.Y = xy$pos.y,
                Group = rep(name, dim(xy)[1]),
                Source = rep("Estimates", dim(xy)[1]))
            dat.list <- rbind(dat.list, dat)
        }
    }
}
plt <- qplot(x = Position.X, y = Position.Y, data = dat.list,
    group = Group, color = Source, linetype= Source, geom = "path")
pdf("pf.pdf", width = 12.8, height = 7.2)
print(plt)
for (plt in plt.list) print(plt)
garbage <- dev.off()
