export ZSH="$HOME/.oh-my-zsh"
export HISTFILE=$HOME/uebk/shell/.zsh_history
ZSH_THEME="robbyrussell"
plugins=(git zsh-autosuggestions zsh-syntax-highlighting fast-syntax-highlighting)

source $ZSH/oh-my-zsh.sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
source /opt/ros/iron/setup.zsh
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

# Setup Gazebo env
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/vscode/ardupilot_gazebo/build/:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=/home/vscode/ardupilot_gazebo/models:/home/vscode/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
