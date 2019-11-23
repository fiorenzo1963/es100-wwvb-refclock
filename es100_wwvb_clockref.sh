#!/bin/bash
ps auxww | grep es100_wwvb_clockref.py | grep -v grep
pid_list=`ps auxww | grep es100_wwvb_clockref.py | grep -v grep | awk ' { print $2; } '`
echo current pid: $pid_list
set -x
kill $pid_list > /dev/null 2>&1
sleep 1
kill -9 $pid_list > /dev/null 2>&1
nohup ./es100_wwvb_clockref.py >> es100_wwvb_clockref.log 2>&1 &
