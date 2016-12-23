#!/bin/bash

var1=hello
var2=world
var3=sung

while echo "Type a variable name:"
      read variable_name
do
    case "$variable_name" in
    "") break ;;
    *) eval echo \$$variable_name ;;
    esac
done
