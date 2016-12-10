#!/bin/bash

to_parse=`df`

echo ${to_parse}
echo ""
echo --------------------------------
echo ""

# I'm going to parse the first line, which contains the column names.
# Find the matches, convert the ENTIRE line into commands that set those matches to the variables, and eval it to actually perform it.
filename=""
used=""
eval `df | sed -n 's/\(File[^ ]*\).*\(Used\).*/filename=\1 used=\2/p'`

echo ${filename}
echo ${used}
