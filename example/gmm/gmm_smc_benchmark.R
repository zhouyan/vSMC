# ============================================================================
#  vSMC/example/gmm/gmm_smc_benchmark.R
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

Process.Args <- function (arg.name, var.name, default)
{
    args <- commandArgs()
    idx <- grep(arg.name, args)
    if (length(idx) > 0) {
        .GlobalEnv[[var.name]] <- max(
            c(1, as.integer(args[idx[length(idx)] + 1])))
    } else {
        .GlobalEnv[[var.name]] <- default
    }
}

Process.Args("--nrepeat", "N.Repeat", 3)
Process.Args("--pnum_smp_lo", "Particle.Number.SMP.Lo", 3)
Process.Args("--pnum_smp_hi", "Particle.Number.SMP.Hi", 14)
Process.Args("--pnum_ocl_lo", "Particle.Number.OCL.Lo", 9)
Process.Args("--pnum_ocl_hi", "Particle.Number.OCL.Hi", 20)

Particle.Number.SMP <- 2^(Particle.Number.SMP.Lo:Particle.Number.SMP.Hi)
Particle.Number.OCL <- 2^(Particle.Number.OCL.Lo:Particle.Number.OCL.Hi)

if (!exists("ExePath")) {
    if (.Platform$OS.type == "unix") ExePath <- "./" else ExePath <- "./Release/"
}
if (!exists("ExeSuffix")) {
    if (.Platform$OS.type == "unix") ExeSuffix <- "" else ExeSuffix <- ".exe"
}

if (.Platform$OS.type == "unix") Redirect <- " 2>&1 " else Redirect <- " "

if (!exists("Force.Run.Benchmark")) Force.Run.Benchmark <- FALSE

if (!exists("Line.Width")) Line.Width <- 80

##############################################################################

SMP.Name <- c(
    seq  = "Sequential",
    tbb  = "Intel TBB",
    gcd  = "Apple GCD",
    ppl  = "Microsoft PPL",
    cilk = "Intel Cilk Plus",
    omp  = "OpenMP",
    std  = "C++11 <thread>")

OCL.Name <- c(
    amd.cpu        = "AMD CPU",
    amd.gpu.32     = "AMD GPU (32)",
    amd.gpu.64     = "AMD GPU (64)",
    apple.cpu      = "Apple CPU",
    apple.agpu.32  = "Apple AMD GPU (32)",
    apple.agpu.64  = "Apple AMD GPU (64)",
    apple.igpu.32  = "Apple Intel GPU (32)",
    apple.igpu.64  = "Apple Intel GPU (64)",
    apple.ngpu.32  = "Apple NVIDIA GPU (32)",
    apple.ngpu.64  = "Apple NVIDIA GPU (64)",
    intel.cpu      = "Intel CPU",
    intel.gpu.32   = "Intel GPU (32)",
    intel.gpu.64   = "Intel GPU (64)",
    nvidia.gpu.32  = "NVIDIA GPU (32)",
    nvidia.gpu.64  = "NVIDIA GPU (64)")

##############################################################################

SMC.Option.String <- function (particle_num)
{paste(" --particle_num", particle_num, "--prior2 100 --complex_model 0 ")}

OCL.Option.String <- function (
    platform_name = "vSMCOpenCLDefault",
    device_type   = "vSMCOpenCLDefault",
    device_vendor = "vSMCOpenCLDefault",
    fp_type_bits  = 64)
{
    paste(
        " ",
        "--cl_platform_name", platform_name,
        "--cl_device_type",   device_type,
        "--cl_device_vendor", device_vendor,
        "--cl_fp_type_bits",  fp_type_bits,
        "--cl_build_option -cl-fast-relaxed-math ")
}

##############################################################################

OCL.Option <- c(
    amd.cpu   = OCL.Option.String("AMD",   "CPU"),
    apple.cpu = OCL.Option.String("Apple", "CPU"),
    intel.cpu = OCL.Option.String("Intel", "CPU"),

    amd.gpu.32     = OCL.Option.String("AMD",    "GPU", "AMD",    32),
    amd.gpu.64     = OCL.Option.String("AMD",    "GPU", "AMD",    64),
    apple.agpu.32  = OCL.Option.String("Apple",  "GPU", "AMD",    32),
    apple.agpu.64  = OCL.Option.String("Apple",  "GPU", "AMD",    64),
    apple.igpu.32  = OCL.Option.String("Apple",  "GPU", "Intel",  32),
    apple.igpu.64  = OCL.Option.String("Apple",  "GPU", "Intel",  64),
    apple.ngpu.32  = OCL.Option.String("Apple",  "GPU", "NVIDIA", 32),
    apple.ngpu.64  = OCL.Option.String("Apple",  "GPU", "NVIDIA", 64),
    intel.gpu.32   = OCL.Option.String("Intel",  "GPU", "Intel",  32),
    intel.gpu.64   = OCL.Option.String("Intel",  "GPU", "Intel",  64),
    nvidia.gpu.32  = OCL.Option.String("NVIDIA", "GPU", "NVIDIA", 32),
    nvidia.gpu.64  = OCL.Option.String("NVIDIA", "GPU", "NVIDIA", 64))

##############################################################################

if (!exists("Bench.SMP")) {
    Bench.SMP <- character()
    for (exe in names(SMP.Name)) {
        if (file.exists(paste(ExePath, "gmm_smc_", exe, ExeSuffix, sep = ""))) {
            Bench.SMP <- c(Bench.SMP, exe)
        }
    }
}

##############################################################################

OCL.TryRun <- function (exe)
{
    output <- system(paste(ExePath, "gmm_smc_cl", ExeSuffix,
            SMC.Option.String(1024), OCL.Option[exe], "--cl_local_size 1",
            Redirect, sep = ""),
        intern = TRUE, ignore.stdout = TRUE)

    timed <- FALSE
    error <- FALSE
    for (line in output) {
        if (length(grep("time.model.order", line)) > 0) timed <- TRUE
        if (length(grep("Runtime Error",    line)) > 0) error <- TRUE
        if (length(grep("Failed to setup",  line)) > 0) error <- TRUE
    }

    timed && !error
}

if (!exists("Bench.OCL")) {
    Bench.OCL <- character()
    if (file.exists(paste(ExePath, "gmm_smc_cl", ExeSuffix, sep = ""))) {
        for (exe in names(OCL.Name)) {
            if (OCL.TryRun(exe)) Bench.OCL <- c(Bench.OCL, exe)
        }
    }
}

##############################################################################

cat(rep("=", Line.Width), "\n", sep = "")
cat("Benchmark setting\n")
cat(rep("-", Line.Width), "\n", sep = "")
for (exe in Bench.SMP) cat(SMP.Name[exe], "\n", sep = "")
for (exe in Bench.OCL) cat(OCL.Name[exe], "\n", sep = "")
cat(rep("=", Line.Width), "\n", sep = "")

##############################################################################

Scale.X <- scale_x_log10(breaks = 10^(1:50),
    labels = math_format()(1:50))
Scale.Y.10 <- scale_y_log10(breaks = 10^(-10:10),
    labels = math_format()(-10:10))
Scale.Y.2 <- scale_y_continuous(breaks = 2^(-10:10),
    labels = math_format(2^.x)(-10:10), trans = log2_trans())

Lab <- xlab("Number of particles")
Theme <- theme_bw(base_size = 24) + theme(
    legend.position = "top", legend.direction = "horizontal")
Guides <- guides(color = guide_legend(ncol = 4))
Width  <- 25.6
Height <- 14.4

##############################################################################

Create.List <- function(particle.number, implementation,
    timefile, nrepeat = 1, legend.nrow = 0)
{
    Time.All <- numeric()
    for (i in 1:nrepeat) {
        Time.All <- cbind(Time.All,
            read.table(paste(timefile, i, sep = "."), header = TRUE)$Time)
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
    cat(rep("=", Line.Width), "\n", sep = "")
    cat(toupper(name), " Benchmark\n", sep = "")
    name <- tolower(name)

    for (r in 1:nrepeat) {
        cat(rep("-", Line.Width), "\n", sep = "")
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
                                    ExePath, "gmm_smc_", exe, ExeSuffix,
                                    SMC.Option.String(n),
                                    Redirect, sep = ""),
                                intern = TRUE, ignore.stdout = TRUE))[3])
                }

                if (!is.na(OCL.Name[exe])) {
                    sink("time.implementation", append = TRUE)
                    cat("\"", OCL.Name[exe], "\"\n", sep = "")
                    sink()
                    Time <- c(Time, system.time(time.tmp <- system(paste(
                                    ExePath, "gmm_smc_cl", ExeSuffix,
                                    SMC.Option.String(n), OCL.Option[exe],
                                    Redirect, sep = ""),
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

        file.rename("time.implementation",
            paste("time.implementation", name, sep = "."))
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

        pdf(File.Name("smp", "time", "pdf", suffix),
            width = Width, height = Height)
        print(smp.list$Time.Plot)
        GC <- dev.off()

        pdf(File.Name("smp", "speedup", "pdf", suffix),
            width = Width, height = Height)
        print(smp.list$Speedup.Plot)
        GC <- dev.off()
    }

    if (length(ocl.list) > 0) {
        save(ocl.list, file = File.Name("ocl", "perf", "RData", suffix))

        pdf(File.Name("ocl", "time", "pdf", suffix),
            width = Width, height = Height)
        print(ocl.list$Time.Plot)
        GC <- dev.off()

        pdf(File.Name("ocl", "speedup", "pdf", suffix),
            width = Width, height = Height)
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

    pdf("Benchmark.Running.pdf", width = Width, height = Height)
    Print.Benchmark.List(Benchmark.Running.List)
    GC <- dev.off()

    pdf("Benchmark.Elapsed.pdf", width = Width, height = Height)
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
