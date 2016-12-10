#!/bin/bash

old_ifs=${IFS}
IFS=:
set x `stty -g`

IFS="${old_ifs}"

echo "dummy variable x: ${1}"
echo "first stty output: ${2}"
echo "16th stty output: ${17}"
