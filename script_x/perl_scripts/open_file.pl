#!/usr/bin/perl
# This script demonstrates the usage of Perl's "filehandlers", which are used 
# by Perl to communicate with filesystem.

use strict;
use warnings;

print "enter the file name: ";
my $file_name=<STDIN>;

# Use Perl's filehandler.
open (IN, $file_name) || die "can't open $file_name: $!\n";

# Use Perl's capability to store things into default, implicit place "$_" 
# when no target point is give when processing.
while (<IN>)
{
    print;
}
close (IN);
