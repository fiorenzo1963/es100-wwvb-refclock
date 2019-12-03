#!/bin/bash
file=$1
if [ ! -r "$file" ]
then
	echo $0: cannot read log file $file
	exit 1
fi
grep "RX_WWVB_CLOCKSTATS," $file | grep RX_OK_ANT | awk -F ',' ' { print $12 } ' > /tmp/out.$$
tools/adev1 60 1 2 < /tmp/out.$$
