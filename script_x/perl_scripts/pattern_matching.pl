#!/usr/bin/perl

$some_var="this var contains some text";

if ($some_var =~ /contains some text/)
{
    print "the variable contains the text.\n";
}
else
{
    print "the variable DOES NOT contain the text.\n";
}

$stdin_text=<>;
print "You typed: $stdin_text\n";
if ($stdin_text =~ /magic keyword/)
{
    print "yes!!!\n";
}
elsif ($stdin_text =~ /(pattern)/)
{
    # The parenthesis between the pattern means capture the found pattern in 
    # $1. Just like shell programming!
    print "pattern captured: $1\n";
}
else
{
    print "no.....\n";
}
