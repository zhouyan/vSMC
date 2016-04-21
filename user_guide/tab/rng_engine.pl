#!/usr/bin/perl

use v5.16.0;

my $engine_std;
$engine_std .= "mt19937 mt19937_64";
$engine_std .= "minstd_rand0 minstd_rand";
$engine_std .= "ranlux24_base ranlux48_base";
$engine_std .= "ranlux24 ranlux48";
$engine_std .= "knuth_b";

my $family;
$family .= " std";
$family .= " philox";
$family .= " threefry";
$family .= " aes128";
$family .= " aes192";
$family .= " aes256";
$family .= " ars";
$family .= " rdrand";
$family .= " mkl";

open RAWFILE, '<', "rng_engine_clang.txt";
my @raw_clang = <RAWFILE>;

open RAWFILE, '<', "rng_engine_gcc.txt";
my @raw_gcc = <RAWFILE>;

open RAWFILE, '<', "rng_engine_intel.txt";
my @raw_intel = <RAWFILE>;

foreach (split ' ', $family) {
    my $tex_clang = &print_tex($_, @raw_clang);
    my $tex_gcc = &print_tex($_, @raw_gcc);
    my $tex_intel = &print_tex($_, @raw_intel);
    my $table = &print_table($tex_clang, $tex_gcc, $tex_intel);
    my $texfile = "rng_engine_$_.tex";
    open TEXFILE, '>', $texfile;
    print TEXFILE $table;
    close TEXFILE;
}

sub print_table
{
    my @rng;
    my @cpB1;
    my @cpB2;
    my $wid = 0;
    foreach my $tex (@_) {
        my @lines = split "\n", $tex;
        while (my ($index, $val) = each @lines) {
            my @record = split ' ', $val;
            $rng[$index] = $record[0];
            $cpB1[$index] .= &print_cpB($record[2]);
            $cpB2[$index] .= &print_cpB($record[4]);
            if ($wid < length($rng[-1])) {
                $wid = length($rng[-1]);
            }
        }
    }

    my $table;
    $table .= '\tbfigures' . "\n";
    $table .= '\begin{tabularx}{\textwidth}{p{2in}XXXXXX}' . "\n";
    $table .= ' ' x 2 . '\toprule' . "\n";
    $table .= ' ' x 2;
    $table .= '& \multicolumn{3}{c}{Loop} ';
    $table .= '& \multicolumn{3}{c}{\verb|rng_rand|} ' . "\\\\\n";
    $table .= ' ' x 2;
    $table .= '\cmidrule(lr){2-4}\cmidrule(lr){5-7}' . "\n";
    $table .= ' ' x 2;
    $table .= '\rng & \llvm & \gnu & Intel & \llvm & \gnu & Intel ' . "\\\\\n";
    $table .= ' ' x 2 . '\midrule' . "\n";
    while (my ($index, $val) = each @rng) {
        $table .= ' ' x 2;
        $table .= sprintf "%-${wid}s", $rng[$index];
        $table .= $cpB1[$index];
        $table .= $cpB2[$index];
        $table .= " \\\\\n";
    }
    $table .= ' ' x 2 . '\bottomrule' . "\n";
    $table .= '\end{tabularx}' . "\n";
    $table
}

sub print_tex
{
    my $engine = shift @_;
    my @rng;
    my @cpB;
    my $wid = 0;
    foreach (@_) {
        next if (!/Passed/);

        my @raw = split;
        my ($rng, $cpB1, $cpB2) = ($raw[0], $raw[5], $raw[6]);
        if ($engine =~ /std/i) {
            next if (!($engine_std =~ /$rng/i));
        } elsif ($engine =~ /mkl/i) {
            next if (!($rng =~ /mkl/i));
        } else {
            next if (!($rng =~ /$engine/i));
            next if ($rng =~ /mkl/i);
        }

        push @rng , '\verb|' . $rng . '|';
        push @cpB , &print_cpB($cpB1) . &print_cpB($cpB2);
        if ($wid < length($rng[-1])) {
            $wid = length($rng[-1]);
        }
    }

    my $tex;
    while (my ($index, $val) = each @cpB) {
        $tex .= ' ' x 2;
        $tex .= sprintf "%-${wid}s", $rng[$index];
        $tex .= $cpB[$index];
        $tex .= " \\\\\n";
    }
    $tex;
}

sub print_cpB
{
    my $cpB = $_[0];
    my $tex = ' & ';
    if ($cpB > 100) {
        $tex .= sprintf "%-4.0f", $cpB;
    } elsif ($cpB > 10) {
        $tex .= sprintf "%-4.1f", $cpB;
    } elsif ($cpB > 1) {
        $tex .= sprintf "%-4.2f", $cpB;
    } else {
        $tex .= sprintf "%-4.2f", $cpB;
    }
    $tex
}
