#! /bin/bash

if [ ! -f $1 ];then
    tail -f /dev/null
else
    echo -e "\033[41;37m"
    echo '============================================='
    echo "bagfile is existed , change dir or delete it"
    echo "bagfile path: $1"
    echo '============================================='
    echo -e "\033[0m"
fi