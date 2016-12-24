#!/usr/bin/perl

# declaration of an array. Seems like keyword "my" is like declaring the variable 
# as a local variable.
my @my_array = (1,2,3,4,5,6,7);

# In order loop through each element, variable that contains the array element 
# is declared, and the entire array is given.
foreach my $element (@my_array)
{
    print "selected $element\n";
}
