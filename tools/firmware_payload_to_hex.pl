#!/usr/bin/perl -w

use strict;

my $file_content = do { local $/; <STDIN> };

$file_content =~ s!^ *\[!!;
$file_content =~ s!\][ \r\n]*$!!;

my @chars = split(/,/, $file_content);
foreach my $char (@chars) {
	print pack("C", $char);
}
