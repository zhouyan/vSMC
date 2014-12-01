# ============================================================================
#  vSMC/example/benchmark.R
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
suppressPackageStartupMessages(library(grid))
suppressPackageStartupMessages(library(scales))

##############################################################################

Process.Args <- function (arg.name, var.name)
{
    args <- commandArgs()
    idx <- grep(arg.name, args)
    if (length(idx) > 0) {
        .GlobalEnv[[var.name]] <- max(
            c(1, as.integer(args[idx[length(idx)] + 1])))
    }
}

Process.Args("--nrepeat", "N.Repeat")
Process.Args("--pnum_smp_lo", "Particle.Number.SMP.Lo")
Process.Args("--pnum_smp_hi", "Particle.Number.SMP.Hi")
Process.Args("--pnum_ocl_lo", "Particle.Number.OCL.Lo")
Process.Args("--pnum_ocl_hi", "Particle.Number.OCL.Hi")

if (!exists("N.Repeat")) N.Repeat <- 3
if (!exists("Particle.Number.SMP.Lo")) Particle.Number.SMP.Lo <- 3
if (!exists("Particle.Number.SMP.Hi")) Particle.Number.SMP.Hi <- 14
if (!exists("Particle.Number.OCL.Lo")) Particle.Number.OCL.Lo <- 9
if (!exists("Particle.Number.OCL.Hi")) Particle.Number.OCL.Hi <- 20

if (!exists("Particle.Number.SMP")) Particle.Number.SMP <-
    2^(Particle.Number.SMP.Lo:Particle.Number.SMP.Hi)
if (!exists("Particle.Number.OCL")) Particle.Number.OCL <-
    2^(Particle.Number.OCL.Lo:Particle.Number.OCL.Hi)

if (!exists("EXECUTABLE_PATH")) {
    if (.Platform$OS.type == "unix") EXECUTABLE_PATH <- "."
    else EXECUTABLE_PATH <- "./Release"
}
if (!exists("EXECUTABLE_SUFFIX")) {
    if (.Platform$OS.type == "unix") EXECUTABLE_SUFFIX <- ""
    else EXECUTABLE_SUFFIX <- ".exe"
}
if (!exists("EXECUTABLE_BASE")) {
    EXECUTABLE_BASE <- ""
    if (file.exists(paste(
                EXECUTABLE_PATH, "/gmm_smc_seq",
                EXECUTABLE_SUFFIX, sep = ""))) {
        EXECUTABLE_BASE <- "gmm"
    }
    if (file.exists(paste(
                EXECUTABLE_PATH, "/node_smc_seq",
                EXECUTABLE_SUFFIX, sep = ""))) {
        EXECUTABLE_BASE <- "node"
    }
    if (file.exists(paste(
                EXECUTABLE_PATH, "/pet_smc_seq",
                EXECUTABLE_SUFFIX, sep = ""))) {
        EXECUTABLE_BASE <- "pet"
    }
}
if (!exists("Force.Run.Benchmark")) Force.Run.Benchmark <- FALSE

##############################################################################

SMP.Name <- c(
    tbb  = "Intel TBB",
    gcd  = "Apple GCD",
    ppl  = "Microsoft PPL",
    cilk = "Intel Cilk Plus",
    omp  = "OpenMP",
    std  = "C++11 <thread>")

OCL.Name <- c(
    amd.cpu       = "AMD CPU",
    amd.gpu.32    = "AMD GPU (32)",
    amd.gpu.64    = "AMD GPU (64)",
    apple.cpu     = "Apple CPU",
    apple.gpu.32  = "Apple GPU (32)",
    apple.gpu.64  = "Apple GPU (64)",
    apple.igpu.32 = "Apple iGPU (32)",
    apple.igpu.64 = "Apple iGPU (64)",
    intel.cpu     = "Intel CPU",
    intel.gpu.32  = "Intel GPU (32)",
    intel.gpu.64  = "Intel GPU (64)",
    nvidia.gpu.32 = "NVIDIA GPU (32)",
    nvidia.gpu.64 = "NVIDIA GPU (64)")

OCL.Platform.Name <- c(
    amd.cpu       = "AMD",
    amd.gpu.32    = "AMD",
    amd.gpu.64    = "AMD",
    apple.cpu     = "Apple",
    apple.gpu.32  = "Apple",
    apple.gpu.64  = "Apple",
    apple.igpu.32 = "Apple",
    apple.igpu.64 = "Apple",
    intel.cpu     = "Intel",
    intel.gpu.32  = "Intel",
    intel.gpu.64  = "Intel",
    nvidia.gpu.32 = "NVIDIA",
    nvidia.gpu.64 = "NVIDIA")

OCL.Device.Name <- c(
    amd.cpu       = "CPU",
    amd.gpu.32    = "GPU",
    amd.gpu.64    = "GPU",
    apple.cpu     = "CPU",
    apple.gpu.32  = "GPU",
    apple.gpu.64  = "GPU",
    apple.igpu.32 = "GPU",
    apple.igpu.64 = "GPU",
    intel.cpu     = "CPU",
    intel.gpu.32  = "GPU",
    intel.gpu.64  = "GPU",
    nvidia.gpu.32 = "GPU",
    nvidia.gpu.64 = "GPU")

OCL.Bits.Name <- c(
    amd.cpu       = "64",
    amd.gpu.32    = "32",
    amd.gpu.64    = "64",
    apple.cpu     = "64",
    apple.gpu.32  = "32",
    apple.gpu.64  = "64",
    apple.igpu.32 = "32",
    apple.igpu.64 = "64",
    intel.cpu     = "64",
    intel.gpu.32  = "32",
    intel.gpu.64  = "64",
    nvidia.gpu.32 = "32",
    nvidia.gpu.64 = "64")

OCL.Vendor.Name <- c(
    amd.cpu       = "Intel",
    amd.gpu.32    = "AMD",
    amd.gpu.64    = "AMD",
    apple.cpu     = "Intel",
    apple.gpu.32  = "NVIDIA",
    apple.gpu.64  = "NVIDIA",
    apple.igpu.32 = "Intel",
    apple.igpu.64 = "Intel",
    intel.cpu     = "Intel",
    intel.gpu.32  = "Intel",
    intel.gpu.64  = "Intel",
    nvidia.gpu.32 = "NVIDIA",
    nvidia.gpu.64 = "NVIDIA")

if (!exists("Bench.SMP")) {
    Bench.SMP <- character()
    for (exe in names(SMP.Name)) {
        if (file.exists(paste(
                    EXECUTABLE_PATH, "/",
                    EXECUTABLE_BASE, "_smc_", exe,
                    EXECUTABLE_SUFFIX, sep = ""))) {
            Bench.SMP <- c(Bench.SMP, exe)
        }
    }
}

if (!exists("Bench.OCL")) {
    if (file.exists(paste(
                EXECUTABLE_PATH, "/",
                EXECUTABLE_BASE, "_smc_cl",
                EXECUTABLE_SUFFIX, sep = ""))) {
        if (length(grep("darwin", version$os)) > 0) Bench.OCL <-
            c("tbb", "apple.igpu.32", "apple.gpu.32")
        else Bench.OCL <- c("tbb")
    } else {
        Bench.OCL <- character()
    }
}

##############################################################################

Scale.X    <- scale_x_log10     (breaks = 10^(1:50),   labels = math_format()(1:50))
Scale.Y.10 <- scale_y_log10     (breaks = 10^(-10:10), labels = math_format()(-10:10))
Scale.Y.2  <- scale_y_continuous(breaks = 2^(-10:10),  labels = math_format(2^.x)(-10:10),
                                 trans = log2_trans())

Lab <- xlab("Number of particles")
Theme <- theme_bw() + theme(legend.position = "top", legend.direction = "horizontal")
Guides <- guides(color = guide_legend(ncol = 6))

##############################################################################

Create.List <- function(particle.number, implementation,
    timefile, nrepeat = 1, legend.nrow = 0)
{
    Time.All <- numeric()
    for (i in 1:nrepeat) {
        Time.All <- cbind(Time.All, read.table(paste(timefile, i, sep = "."), header = TRUE)$Time)
    }

    Time <- numeric()
    for (i in 1:dim(Time.All)[1]) Time <- c(Time, min(Time.All[i,]))

    N.Perf.Data <- length(Time)
    N.Particle.Number <- length(particle.number)
    N.Implementation <- N.Perf.Data / N.Particle.Number
    Implementation.Levels <- unique(as.character(implementation))
    Implementation <- ordered(implementation, levels = Implementation.Levels)
    Particle.Number <- rep(particle.number, each = N.Implementation)

    Speedup <- rep(Time[seq(1, N.Perf.Data, N.Implementation)],
        each = N.Implementation) / Time
    Speedup[!is.finite(Speedup)] <- 1

    Perf.Data <- data.frame(
        Time = Time,
        Particle.Number = Particle.Number,
        Implementation = Implementation,
        Speedup = Speedup)

    Aes <- aes(
        x = Particle.Number,
        group = Implementation,
        color = Implementation,
        shape = Implementation)

    Time.Plot <- ggplot(Perf.Data) + aes(y = Time)
    Time.Plot <- Time.Plot + geom_line() + geom_point()
    Time.Plot <- Time.Plot + Aes + Scale.X + Scale.Y.10 + Lab
    Time.Plot <- Time.Plot + ylab("Wall clock time (second)")
    Time.Plot <- Time.Plot + Theme + Guides

    Speedup.Plot <- ggplot(Perf.Data) + aes(y = Speedup)
    Speedup.Plot <- Speedup.Plot + geom_line() + geom_point()
    Speedup.Plot <- Speedup.Plot + Aes + Scale.X + Scale.Y.2 + Lab
    Speedup.Plot <- Speedup.Plot + ylab("Speedup")
    Speedup.Plot <- Speedup.Plot + Theme

    list(Perf.Data = Perf.Data,
        Time.Plot = Time.Plot, Speedup.Plot = Speedup.Plot)
}

##############################################################################

VP.Layout <- function (x, y) viewport(layout.pos.row = x, layout.pos.col = y)

Print.Plot.List <- function (plot.list)
{
    grid.newpage()
    pushViewport(viewport(layout = grid.layout(2, 1)))
    print(plot.list$Time.Plot,    vp = VP.Layout(1, 1))
    print(plot.list$Speedup.Plot, vp = VP.Layout(2, 1))
}

Print.Benchmark.List <- function (benchmark.list)
{
    smp.plot.list <- benchmark.list$SMP.List
    ocl.plot.list <- benchmark.list$OCL.List

    if (length(smp.plot.list) > 0 && length(ocl.plot.list) > 0) {
        grid.newpage()
        pushViewport(viewport(layout = grid.layout(2, 2)))
        print(smp.plot.list$Time.Plot,    vp = VP.Layout(1, 1))
        print(smp.plot.list$Speedup.Plot, vp = VP.Layout(2, 1))
        print(ocl.plot.list$Time.Plot,    vp = VP.Layout(1, 2))
        print(ocl.plot.list$Speedup.Plot, vp = VP.Layout(2, 2))
    } else if (length(smp.plot.list) > 0) {
        grid.newpage()
        pushViewport(viewport(layout = grid.layout(2, 1)))
        print(smp.plot.list$Time.Plot,    vp = VP.Layout(1, 1))
        print(smp.plot.list$Speedup.Plot, vp = VP.Layout(2, 1))
    } else if (length(ocl.plot.list) > 0) {
        grid.newpage()
        pushViewport(viewport(layout = grid.layout(2, 1)))
        print(ocl.plot.list$Time.Plot,    vp = VP.Layout(1, 1))
        print(ocl.plot.list$Speedup.Plot, vp = VP.Layout(2, 1))
    }
}

##############################################################################

Run.Benchmark <- function (particle.number, name, implementation, nrepeat = 1)
{
    cat(rep("=", 63), "\n", sep = "")
    cat(toupper(name), " Benchmark\n", sep = "")
    name <- tolower(name)

    if (.Platform$OS.type == "unix") redirect <- "2>&1"
    else redirect <- ""

    for (r in 1:nrepeat) {
        cat(rep("-", 63), "\n", sep = "")
        cat("Run number: ", r, "\n", sep = "")

        sink(paste("time", name, "running", r, sep = "."))
        cat("time.model.order Time\n", sep = "")
        sink()

        sink("time.implementation")
        cat("Implementation\n", sep = "")
        sink()

        Time <- numeric()
        for (n in particle.number) {
            cat("Number of particles: ",
            "\t2^", log2(n), "\t(", n, ") ... ", sep = "")
            Time.Step <- 0
            for (exe in implementation) {
                if (!is.na(SMP.Name[exe])) {
                    sink("time.implementation", append = TRUE)
                    cat("\"", SMP.Name[exe], "\"\n", sep = "")
                    sink()
                    Time <- c(Time, system.time(time.tmp <- system(paste(
                                    EXECUTABLE_PATH, "/",
                                    EXECUTABLE_BASE, "_smc_", exe,
                                    " --particle_num ", n,
                                    " --prior2 100 --complex_model 0",
                                    " ", redirect, sep = ""),
                                intern = TRUE, ignore.stdout = TRUE))[3])
                }

                if (!is.na(OCL.Name[exe])) {
                    sink("time.implementation", append = TRUE)
                    cat("\"", OCL.Name[exe], "\"\n", sep = "")
                    sink()
                    Time <- c(Time, system.time(time.tmp <- system(paste(
                                    EXECUTABLE_PATH, "/",
                                    EXECUTABLE_BASE, "_smc_cl",
                                    " --particle_num ", n,
                                    " --prior2 100 --complex_model 0",
                                    " --cl_platform_name ",
                                    OCL.Platform.Name[exe],
                                    " --cl_device_type ",
                                    OCL.Device.Name[exe],
                                    " --cl_device_vendor ",
                                    OCL.Vendor.Name[exe],
                                    " --cl_fp_type_bits ",
                                    OCL.Bits.Name[exe],
                                    " ", redirect, sep = ""),
                                intern = TRUE, ignore.stdout = TRUE))[3])
                }

                Time.Step <- Time.Step + Time[length(Time)]

                sink(paste("time", name, "running", r, sep = "."),
                    append = TRUE)
                for (line in time.tmp) {
                    if (length(grep("time.model.order", line)) > 0) {
                        cat(line, "\n", sep = "")
                    }
                }
                sink()

                sink(paste("time.log", name, sep = "."), append = TRUE)
                for (line in time.tmp) cat(line, "\n", sep = "")
                sink()

                sink(paste("time", name, "elapsed", r, sep = "."))
                cat("Time\n", sep = "")
                sink()
                write.table(Time, append = TRUE,
                    file = paste("time", name, "elapsed", r, sep = "."),
                    row.names = FALSE, col.names = FALSE)
            }
            cat("Done ... ", Time.Step, " sec\n", sep = "")
        }

        file.rename("time.implementation", paste("time.implementation", name, sep = "."))
    }
}

##############################################################################

Read.List <- function (particle.number, name, suffix, nrepeat)
{
    implementation <- read.table(paste("time.implementation", name, sep = "."),
        header = TRUE)$Implementation
    Create.List(particle.number, implementation,
        timefile = paste("time", name, suffix, sep = "."),
        nrepeat = nrepeat, legend.nrow = 0)
}

##############################################################################

Combine.List <- function (lists)
{
    Configure <- character()
    for (i in 1:length(lists)) {
        Configure <- c(Configure,
            rep(lists[[i]]$Configure, dim(lists[[i]]$List$Perf.Data)[1]))
    }
    Perf.Data <- lists[[1]]$List$Perf.Data
    for (i in 2:length(lists)) {
        Perf.Data <- rbind(Perf.Data, lists[[i]]$List$Perf.Data)
    }
    Perf.Data$Configure <- Configure

    Aes.Configure <- aes(
        x = Particle.Number,
        group = Configure,
        color = Configure)

    Time.Plot <- ggplot(Perf.Data) + aes(y = Time)
    Time.Plot <- Time.Plot + geom_line() + geom_point()
    Time.Plot <- Time.Plot + Aes.Configure + Scale.X + Scale.Y.10 + Lab
    Time.Plot <- Time.Plot + ylab("Wall clock time (second)")
    Time.Plot <- Time.Plot + facet_wrap(~Implementation)
    Time.Plot <- Time.Plot + Theme + Guides

    list(Perf.Data = Perf.Data, Time.Plot = Time.Plot)
}


##############################################################################

Combine.Benchmark.List <- function (lists)
{
    smp.list <- list()
    ocl.list <- list()
    for (i in 1:length(lists)) {
        smp.list[[i]] <- list(
            Configure = lists[[i]]$Configure, List = lists[[i]]$List$SMP.List)
        ocl.list[[i]] <- list(
            Configure = lists[[i]]$Configure, List = lists[[i]]$List$OCL.List)
    }
    list(SMP.List = Combine.List(smp.list), OCL.List = Combine.List(ocl.list))
}

##############################################################################

File.Name <- function (base, func, suffix, extra = "")
{
    if (extra == "") {
        paste("bench-", base, "-", func, ".", suffix, sep = "")
    } else {
        paste("bench-", base, "-", func, "-", extra, ".", suffix, sep = "")
    }
}

Save.List <- function (benchmark.list, suffix = "")
{
    smp.list <- benchmark.list$SMP.List
    ocl.list <- benchmark.list$OCL.List

    save(benchmark.list, file = File.Name("smpocl", "perf", "RData", suffix))

    if (length(smp.list) > 0) {
        save(smp.list, file = File.Name("smp", "perf", "RData", suffix))

        pdf(File.Name("smp", "time", "pdf", suffix), width = 10, height = 6)
        print(smp.list$Time.Plot)
        GC <- dev.off()

        pdf(File.Name("smp", "speedup", "pdf", suffix), width = 10, height = 6)
        print(smp.list$Speedup.Plot)
        GC <- dev.off()
    }

    if (length(ocl.list) > 0) {
        save(ocl.list, file = File.Name("ocl", "perf", "RData", suffix))

        pdf(File.Name("ocl", "time", "pdf", suffix), width = 10, height = 6)
        print(ocl.list$Time.Plot)
        GC <- dev.off()

        pdf(File.Name("ocl", "speedup", "pdf", suffix), width = 10, height = 6)
        print(ocl.list$Speedup.Plot)
        GC <- dev.off()
    }
}

##############################################################################

if (!interactive() || Force.Run.Benchmark) {
    if (length(Bench.SMP) > 0) Benchmark.Time.SMP <- system.time(Run.Benchmark(
            Particle.Number.SMP, "smp", Bench.SMP, nrepeat = N.Repeat))
    if (length(Bench.OCL) > 0) Benchmark.Time.OCL <- system.time(Run.Benchmark(
            Particle.Number.OCL, "ocl", Bench.OCL, nrepeat = N.Repeat))

    SMP.List <- list()
    OCL.List <- list()
    if (length(Bench.SMP) > 0) SMP.List = Read.List(
        Particle.Number.SMP, "smp", "running", N.Repeat)
    if (length(Bench.OCL) > 0) OCL.List = Read.List(
        Particle.Number.OCL, "ocl", "running", N.Repeat)
    Benchmark.Running.List <- list(SMP.List = SMP.List, OCL.List = OCL.List)

    SMP.List <- list()
    OCL.List <- list()
    if (length(Bench.SMP) > 0) SMP.List = Read.List(
        Particle.Number.SMP, "smp", "elapsed", N.Repeat)
    if (length(Bench.OCL) > 0) OCL.List = Read.List(
        Particle.Number.OCL, "ocl", "elapsed", N.Repeat)
    Benchmark.Elapsed.List <- list(SMP.List = SMP.List, OCL.List = OCL.List)

    Save.List(Benchmark.Running.List, "running")
    Save.List(Benchmark.Elapsed.List, "elapsed")

    pdf("Benchmark.Running.pdf", width = 20, height = 12)
    Print.Benchmark.List(Benchmark.Running.List)
    GC <- dev.off()

    pdf("Benchmark.Elapsed.pdf", width = 20, height = 12)
    Print.Benchmark.List(Benchmark.Elapsed.List)
    GC <- dev.off()

    if (interactive()) {
        if (.Platform$OS.type == "windows") {
            Use.Cairo <- suppressPackageStartupMessages(suppressWarnings(
                    require(cairoDevice)))
        } else {
            Use.Cairo <- FALSE
        }

        if (Use.Cairo) Cairo() else dev.new()
        Print.Benchmark.List(Benchmark.Running.List)

        if (Use.Cairo) Cairo() else dev.new()
        Print.Benchmark.List(Benchmark.Elapsed.List)
    }
}
