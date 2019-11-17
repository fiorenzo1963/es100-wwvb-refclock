#!/bin/bash
ps auxww | grep es100-wwvb-test.py | grep -v grep
set -x
killall es100-wwvb-test.py > /dev/null 2>&1
sleep 1
killall -9 es100-wwvb-test.py > /dev/null 2>&1
nohup ./es100-wwvb-test.py >> es100-wwvb-test.log 2>&1 &
