#!/usr/bin/perl

use v5.16;

do 'tab.pl';

my @std = qw(mt19937 mt19937_64 minstd_rand0 minstd_rand ranlux24_base
ranlux48_base ranlux24 ranlux48 knuth_b);

my @engines = qw(STD Philox Threefry AES128 AES192 AES256 ARS RDRAND MKL);

my @clang_txt = &read('clang');
my @gcc_txt = &read('gcc');
my @intel_txt = &read('intel');

foreach (@engines) {
    open my $texfile, '>', "rng_engine_\L$_.tex";
    print $texfile &table(
        &filter($_, @clang_txt),
        &filter($_, @gcc_txt),
        &filter($_, @intel_txt));
}

sub read
{
    open my $txtfile, '<', "rng_engine_$_[0].txt";
    my @txt = <$txtfile>;
    @txt
}

sub filter
{
    my $engine = shift @_;
    my $record;
    foreach (@_) {
        next if (!/Passed|Failed/);

        my ($rng, $cpB1, $cpB2) = (split)[0, 5, 6];
        if ($engine eq 'STD') {
            next unless "@std" =~ /$rng/;
        } elsif ($engine eq 'MKL') {
            next unless $rng =~ /MKL/;
        } else {
            next unless $rng =~ /$engine/;
            next if $rng =~ /MKL/;
        }

        $record .= $rng . ' ';
        $record .= $cpB1 . ' ';
        $record .= $cpB2 . "\n";
    }
    $record
}

sub table
{
    my @rng;
    my @cpB1;
    my @cpB2;
    my $wid = 0;
    foreach (@_) {
        my @lines = split "\n", $_;
        my $index = 0;
        foreach (@lines) {
            my @record = split;
            $rng[$index] = '\verb|' . $record[0] . '|';
            $cpB1[$index] .= &format($record[1]);
            $cpB2[$index] .= &format($record[2]);
            if ($wid < length($rng[-1])) {
                $wid = length($rng[-1]);
            }
            $index++;
        }
    }

    my $table;
    $table .= '\tbfigures' . "\n";
    $table .= '\begin{tabularx}{\textwidth}{p{1.8in}RRRRRR}' . "\n";
    $table .= ' ' x 2 . '\toprule' . "\n";
    $table .= ' ' x 2;
    $table .= '& \multicolumn{3}{c}{Loop} ';
    $table .= '& \multicolumn{3}{c}{\verb|rand|}';
    $table .= " \\\\\n";
    $table .= ' ' x 2 . '\cmidrule(lr){2-4}\cmidrule(lr){5-7}' . "\n";
    $table .= ' ' x 2 . '\rng & \llvm & \gnu & Intel & \llvm & \gnu & Intel';
    $table .= " \\\\\n";
    $table .= ' ' x 2 . '\midrule' . "\n";
    my $index = 0;
    foreach (@rng) {
        $table .= ' ' x 2;
        $table .= sprintf "%-${wid}s", $rng[$index];
        $table .= $cpB1[$index];
        $table .= $cpB2[$index];
        $table .= " \\\\\n";
        $index++;
    }
    $table .= ' ' x 2 . '\bottomrule' . "\n";
    $table .= '\end{tabularx}' . "\n";
    $table;
}
