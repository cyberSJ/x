#!/bin/bash
BELL=`echo x | tr 'x' '\007'`
sed "s/wo/$BELL/"
