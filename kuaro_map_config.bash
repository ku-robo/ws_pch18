#! /bin/bash

DIR_MAP_GLOBAL="${HOME}/ws_pch18/kuaro_map"
DIR_MAP_LOCAL="/1101/first"

export DIR_MAP="${DIR_MAP_GLOBAL}${DIR_MAP_LOCAL}"
mkdir -p $DIR_MAP

export DIR_MAP_BAG="${DIR_MAP}/bag"
mkdir -p $DIR_MAP_BAG

export DIR_MAP_OUT="${DIR_MAP}/out"
mkdir -p $DIR_MAP_OUT

echo 'haha'