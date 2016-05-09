sub format
{
    my $num = shift @_;
    if ($num > 100) {
        ' & ' . sprintf('%-6.0f', $num);
    } elsif ($num > 10) {
        ' & ' . sprintf('%-6.1f', $num);
    } else {
        ' & ' . sprintf('%-6.2f', $num);
    }
}
