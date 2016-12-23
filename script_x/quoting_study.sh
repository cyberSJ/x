#!/bin/bash -x

for arg in $*
do
  echo "arg is: ${arg}"
done

for arg in $@
do
  echo "arg is: ${arg}"
done

# This is the best quoting.
# This can handle correctly inputs like: "arg with space" arg2 arg3 
for arg in "$@"
do
  echo "arg is: ${arg}"
done
