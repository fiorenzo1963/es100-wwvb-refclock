#!/bin/bash
#
# extract statistics suitable for offset plotting with GNU plot.
#
file=$1
if [ ! -r "$file" ]
then
	echo $0: cannot read log file $file
	exit 1
fi

grep "RX_WWVB_CLOCKSTATS," $file | awk -F ',' ' { off = $12; if (length($12)==0) off = "*"; print $7, $8, off } '
