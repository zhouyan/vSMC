sub format_cpB
{
    my $cpB = $_[0];
    my $line = ' & ';
    if ($cpB > 100) {
        $line .= sprintf '%-4.0f', $cpB;
    } elsif ($cpB > 10) {
        $line .= sprintf '%-4.1f', $cpB;
    } elsif ($cpB > 1) {
        $line .= sprintf '%-4.2f', $cpB;
    } else {
        $line .= sprintf '%-4.2f', $cpB;
    }
    $line;
}
