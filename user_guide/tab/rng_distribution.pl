#!/usr/bin/perl

use v5.16;

do 'tab.pl';

my @inverse = qw(Cauchy Exponential ExtremeValue Laplace Logistic Pareto
Rayleigh UniformReal Weibull);

my @normal = qw(Normal Lognormal Levy);

my @nostd = qw(Logistic Pareto Rayleigh Levy);

my %distribution;
my %cpB;
my $txt;
open my $txtfile, '<', 'rng_distribution.txt';
while (<$txtfile>) {
    if (/<double>\(.*Passed/) {
        $txt .= $_;
        my @record = split;
        my $name = shift @record;
        $name =~ s/(.*)<double>(.*)/$1$2/;
        $name = '\verb|' . $name . "|\n";
        my $basename = $1;
        my $cpB;
        if ("@nostd" =~ /$basename/) {
            shift @record;
            $cpB .= sprintf ' & %-4s', '--';
        } else {
            $cpB .= &format_cpB(shift @record);
        }
        foreach (@record[0..2]) {
            $cpB .= &format_cpB($_)
        }
        $cpB .= "\n";
        if ("@inverse" =~ /$basename/) {
            $distribution{'inverse'} .= $name;
            $cpB{'inverse'} .= $cpB;
        } elsif ("@normal" =~ /$basename/) {
            $distribution{'normal'} .= $name;
            $cpB{'normal'} .= $cpB;
        } else {
            $distribution{$basename} .= $name;
            $cpB{$basename} .= $cpB;
        }
    }
}
open $txtfile, '>', 'rng_distribution.txt';
print $txtfile $txt;

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
    $table .= '\begin{tabularx}{\textwidth}{p{2in}YYYY}' . "\n";
    $table .= ' ' x 2 . '\toprule' . "\n";
    $table .= ' ' x 2;
    $table .= 'Distribution & \std/Boost & \vsmc & \verb|rand| & \mkl';
    $table .= " \\\\\n";
    $table .= ' ' x 2 . '\midrule' . "\n";
    my $index = 0;
    foreach (@dist) {
        $table .= ' ' x 2;
        $table .= sprintf "%-${wid}s", $dist[$index];
        $table .= $cpB[$index];
        $table .= " \\\\\n";
        $index++;
    }
    $table .= ' ' x 2 . '\bottomrule' . "\n";
    $table .= '\end{tabularx}' . "\n";
    open my $texfile, '>', "rng_distribution_\L$basename.tex";
    print $texfile $table;
}
