#!/usr/bin/perl

use v5.16.0;

do 'tab.pl';

my $std = join ' ', qw(
mt19937 mt19937_64
minstd_rand0
minstd_rand
ranlux24_base
ranlux48_base
ranlux24
ranlux48
knuth_b);

my @family = qw(
std
philox
threefry
aes128
aes192
aes256
ars
rdrand
mkl);

foreach (@family) {
    my $clang = &filter($_, 'clang');
    my $gcc = &filter($_, 'gcc');
    my $intel = &filter($_, 'intel');
    my $table = &tex_table($clang, $gcc, $intel);
    my $texfile = "rng_engine_$_.tex";
    open TEXFILE, '>', $texfile;
    print TEXFILE $table;
}

sub tex_table
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
            $cpB1[$index] .= &format_cpB($record[1]);
            $cpB2[$index] .= &format_cpB($record[2]);
            if ($wid < length($rng[-1])) {
                $wid = length($rng[-1]);
            }
            $index += 1;
        }
    }

    my $table;
    $table .= '\tbfigures' . "\n";
    $table .= '\begin{tabularx}{\textwidth}{p{2in}XXXXXX}' . "\n";
    $table .= ' ' x 2 . '\toprule' . "\n";
    $table .= ' ' x 2;
    $table .= '& \multicolumn{3}{c}{Loop} ';
    $table .= '& \multicolumn{3}{c}{\verb|rng_rand|} \\\\' . "\n";
    $table .= ' ' x 2;
    $table .= '\cmidrule(lr){2-4}\cmidrule(lr){5-7}' . "\n";
    $table .= ' ' x 2;
    $table .= '\rng & \llvm & \gnu & Intel & \llvm & \gnu & Intel \\\\' . "\n";
    $table .= ' ' x 2 . '\midrule' . "\n";
    my $index = 0;
    foreach (@rng) {
        $table .= ' ' x 2;
        $table .= sprintf "%-${wid}s", $rng[$index];
        $table .= $cpB1[$index];
        $table .= $cpB2[$index];
        $table .= " \\\\\n";
        $index += 1;
    }
    $table .= ' ' x 2 . '\bottomrule' . "\n";
    $table .= '\end{tabularx}' . "\n";
    $table;
}

sub filter
{
    my ($engine, $compiler) = @_;
    open RAWFILE, '<', "rng_engine_$compiler.txt";
    my @raw = <RAWFILE>;

    my $record;
    foreach (@raw) {
        next if (!/Passed/);

        my @line = split;
        my ($rng, $cpB1, $cpB2) = ($line[0], $line[5], $line[6]);
        if ($engine =~ /std/i) {
            next if (!($std =~ /$rng/i));
        } elsif ($engine =~ /mkl/i) {
            next if (!($rng =~ /mkl/i));
        } else {
            next if (!($rng =~ /$engine/i));
            next if ($rng =~ /mkl/i);
        }

        $record .= $rng . ' ';
        $record .= $cpB1 . ' ';
        $record .= $cpB2 . "\n";
    }
    $record
}
