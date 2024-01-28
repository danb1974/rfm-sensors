#!/usr/bin/perl -w

use strict;

my $file_content = do { local $/; <STDIN> };

my $output = "[";

my @chars = split '', $file_content;
for my $char (@chars) {
	$output .= unpack("C", $char);
	$output .= ",";
}
$output =~ s!,$!!;

$output .= "]";

print "$output\n";
