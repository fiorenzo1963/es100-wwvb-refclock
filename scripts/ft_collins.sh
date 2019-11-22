#!/bin/bash
FT_COLLINS_LAT=40.585258
FT_COLLINS_LONG=-105.084419

#
# argument format: YYYY-MM-DD
#
if [ "$1" != "" ]
then
	DATE="&date=$1"
else
	DATE=""
fi

#
# This API is free. Usage requires attribution. See https://sunrise-sunset.org/api
#
URL="https://api.sunrise-sunset.org/json?lat=$FT_COLLINS_LAT&lng=$FT_COLLINS_LONG$DATE&formatted=0"

curl $URL
echo ''
