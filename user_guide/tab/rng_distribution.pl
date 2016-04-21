#!/usr/bin/perl

use v5.16.0;

do 'tab.pl';

my $normal = join ' ', qw(Normal Lognormal Levy); 

my $inverse = join ' ', qw(Cauchy Exponential ExtremeValue Laplace Logistic
Pareto Rayleigh UniformReal Weibull);

my $nostd = join ' ', qw(Logistic Pareto Rayleigh Levy);

my %distribution;
my %cpB;
open RAWFILE, '<', 'rng_distribution.txt';
while (<RAWFILE>) {
    if (/<double>\(/) {
        s/(.*)<double>(\(.*?\)).*/$1$2/;
        chomp;
        my $name = '\verb|' . $_ . "|\n";
        my $basename = $1;
        $_ = <RAWFILE>;
        $_ = <RAWFILE>;
        my @record = split;
        shift @record;
        my $cpB;
        if ($nostd =~ /$basename/) {
            shift @record;
            $cpB .= sprintf ' & %-4s', '--';
        } else {
            $cpB .= format_cpB(shift @record);
        }
        foreach (@record[0..2]) {
            $cpB .= format_cpB($_);
        }
        $cpB .= "\n";
        if ($normal =~ /$basename/) {
            $distribution{'normal'} .= $name;
            $cpB{'normal'} .= $cpB;
        } elsif ($inverse =~ /$basename/) {
            $distribution{'inverse'} .= $name;
            $cpB{'inverse'} .= $cpB;
        } else {
            $distribution{$basename} .= $name;
            $cpB{$basename} .= $cpB;
        }
    }
}

while (my ($basename, $name) = each %distribution) {
    my @dist = split "\n", $distribution{$basename};
    my @cpB = split "\n", $cpB{$basename};
    my $wid = 0;
    foreach (@dist) {
        if ($wid < length($_)) {
            $wid = length($_);
        }
    }

    my $table;
    $table .= '\tbfigures' . "\n";
    $table .= '\begin{tabularx}{\textwidth}{p{2in}XXXX}' . "\n";
    $table .= ' ' x 2 . '\toprule' . "\n";
    $table .= ' ' x 2;
    $table .= 'Distribution & \std/Boost & \vsmc & \verb|rng_rand| & \mkl \\\\';
    $table .= "\n";
    $table .= ' ' x 2 . '\midrule' . "\n";
    my $index = 0;
    foreach (@dist) {
        $table .= ' ' x 2;
        $table .= sprintf "%-${wid}s", $dist[$index];
        $table .= $cpB[$index];
        $table .= " \\\\\n";
        $index += 1;
    }
    $table .= ' ' x 2 . '\bottomrule' . "\n";
    $table .= '\end{tabularx}' . "\n";
    my $texfile = "rng_distribution_\L$basename.tex";
    open TEXFILE, '>', $texfile;
    print TEXFILE $table;
}
