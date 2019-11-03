DIR_MAP_GLOBAL="${HOME}/ws_pch18/kuaro_map"
DIR_MAP_LOCAL="/1101/first"

export DIR_MAP="${DIR_MAP_GLOBAL}${DIR_MAP_LOCAL}"
export DIR_GATHER=`dirname $BASH_SOURCE`

mkdir -p $DIR_MAP
