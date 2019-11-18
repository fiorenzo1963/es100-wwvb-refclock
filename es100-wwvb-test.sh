#!/bin/bash
ps auxww | grep es100-wwvb-test.py | grep -v grep
pid_list=`ps auxww | grep es100-wwvb-test.py | grep -v grep | awk ' { print $2; } '`
set -x
kill $pid_list > /dev/null 2>&1
sleep 1
kill -9 $pid_list > /dev/null 2>&1
nohup ./es100-wwvb-test.py >> es100-wwvb-test.log 2>&1 &
