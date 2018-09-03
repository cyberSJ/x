#!/bin/bash
# Build the libjfftw.so
gradle build

# Run the custom Java code that uses libjfftw.so
gradle run
