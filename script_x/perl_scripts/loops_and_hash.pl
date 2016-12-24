#!/usr/bin/perl

# some site suggested these to be used in all perl script because it shows 
# more warnings/error of the script.
use strict;
use warnings;

my %my_hash= ( first => 'one',
	       second => 'two',
	       third => 'three' );

# The concept of accessing hash map key/value pair is similar to accessing 
# array element. Instead of providing the value to keep the array element, we 
# provide the key/value set to keep the temporary array element. Then we 
# provide the entire hash map. But there is equal sign in between. But this 
# one is an infinite loop without the "each" keyword.
while (my ($key, $value) = each %my_hash)
{
    print "$key has a value of $value\n";
}

foreach my $key (keys %my_hash)
{
    print "$key\n";
}

# Dereferencing in perl is getting the original-object-TYPE the refernce was 
# referring to. Reference in perl is scalar type. So in order to get a hash 
# map type from a reference that was really pointint to a hash map, you need 
# to dereference the reference. 
# Referencer = \
# Dereference = %{} for hash, @{} for array ${} for scalar.
my $reference_to_hash = \%my_hash;
foreach my $value (values %{$reference_to_hash})
{
   print "value = ", $value, "\n";
}

while ( my $line = <STDIN> )
{
    print "I got $line\n";
}
