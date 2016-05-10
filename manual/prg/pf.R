library(ggplot2)
library(tikzDevice)

pf <- read.table("pf.out", header = TRUE)
sink("pf.rout")
print(pf[1:5,])
sink()

obs <- read.table("pf.data", header = FALSE)
dat <- data.frame(
    X = c(pf[["pos.0"]], obs[,1]),
    Y = c(pf[["pos.1"]], obs[,2]))
dat[["Source"]] <- rep(c("Estimate", "Observation"), each = dim(obs)[1])
plt <- qplot(x = X, y = Y, data = dat, geom = "path")
plt <- plt + aes(group = Source, color = Source, linetype = Source)
plt <- plt + xlab("$X$")
plt <- plt + ylab("$Y$")
plt <- plt + theme_bw() + theme(legend.position = "top")

tikz("pf.tex", width = 5, height = 5, standAlone = TRUE)
print(plt)
dev.off()
system("latexmk -silent -f pf.tex &>/dev/null")
system("latexmk -c pf.tex &>/dev/null")
system("rm -f pf.tex")
system("cp pf.pdf ../fig/.")
