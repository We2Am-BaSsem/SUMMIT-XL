prefix="ros-noetic"
# List of packages we need 
packages=("navigation" "gmapping" "robot-localization" "mavros-msgs" "velocity-controllers" "twist-mux" "teleop-twist-keyboard")

# Base install command
apt_command="sudo apt-get install "

# Build a long command that should look like 
# `sudo apt-get install ros-noetic-pkg1 ros-noetic-pkg2 ... ros-noetic-pkgn -y`
for pkg in ${packages[@]}; do
	apt_command+="$prefix-$pkg "
done
apt_command+=" -y"

# Run the command
eval $apt_command

make_project

source ./devel/setup.bash

echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
