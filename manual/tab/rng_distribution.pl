#!/usr/bin/perl

use v5.16;

do 'tab.pl';

my @inverse = qw(Arcsine Cauchy Exponential ExtremeValue Laplace Logistic
Pareto Rayleigh UniformReal Weibull);

my @normal = qw(Normal Lognormal Levy);

my @nostd = qw(Arcsine, Logistic Pareto Rayleigh Levy);

my %distribution;
my %cpE;
my $txt;
open my $txtfile, '<', 'rng_distribution.txt';
while (<$txtfile>) {
    if (/<double>\(.*(Passed|Failed)/) {
        $txt .= $_;
        my @record = split;
        my $name = shift @record;
        $name =~ s/(.*)<double>(.*)/$1$2/;
        $name = '\verb|' . $name . "|\n";
        my $basename = $1;
        my $cpE;
        if ("@nostd" =~ /$basename/) {
            shift @record;
            $cpE .= sprintf ' & %-4s', '--';
        } else {
            $cpE .= &format(shift @record);
        }
        foreach (@record[0..2]) {
            $cpE .= &format($_)
        }
        $cpE .= "\n";
        if ("@inverse" =~ /$basename/) {
            $distribution{'inverse'} .= $name;
            $cpE{'inverse'} .= $cpE;
        } elsif ("@normal" =~ /$basename/) {
            $distribution{'normal'} .= $name;
            $cpE{'normal'} .= $cpE;
        } else {
            $distribution{$basename} .= $name;
            $cpE{$basename} .= $cpE;
        }
    }
}
open $txtfile, '>', 'rng_distribution.txt';
print $txtfile $txt;

while (my ($basename, $name) = each %distribution) {
    my @dist = split "\n", $distribution{$basename};
    my @cpE = split "\n", $cpE{$basename};
    my $wid = 0;
    foreach (@dist) {
        if ($wid < length($_)) {
            $wid = length($_);
        }
    }

    my $table;
    $table .= '\tbfigures' . "\n";
    $table .= '\begin{tabularx}{\textwidth}{p{1.8in}RRRR}' . "\n";
    $table .= ' ' x 2 . '\toprule' . "\n";
    $table .= ' ' x 2;
    $table .= 'Distribution & \std & \vsmc & \verb|rand| & \mkl';
    $table .= " \\\\\n";
    $table .= ' ' x 2 . '\midrule' . "\n";
    my $index = 0;
    foreach (@dist) {
        $table .= ' ' x 2;
        $table .= sprintf "%-${wid}s", $dist[$index];
        $table .= $cpE[$index];
        $table .= " \\\\\n";
        $index++;
    }
    $table .= ' ' x 2 . '\bottomrule' . "\n";
    $table .= '\end{tabularx}' . "\n";
    open my $texfile, '>', "rng_distribution_\L$basename.tex";
    print $texfile $table;
}
