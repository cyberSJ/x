#!/bin/bash -x

case "${1}/${2}/${3}" in

	1/2/3) echo "yey";;
	3/2/1) echo "back";;
	3/2/*) echo "three two ${3}";;
	*) echo "last case";;
esac
