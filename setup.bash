
source ~/ws_cartographer/install_isolated/setup.bash
alias cm-cartographer='cd ~/ws_cartographer && catkin_make_isolated --install --use-ninja'

source `dirname ${BASH_SOURCE}`/devel/setup.bash
alias cm-pch18="cd `dirname ${BASH_SOURCE}` && catkin_make"

export WS_CONFIG_PATH="`dirname ${BASH_SOURCE}`/kuaro_map_config.bash"
alias roslaunch="source ${WS_CONFIG_PATH} && roslaunch"

