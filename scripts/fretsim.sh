my_dir=$(dirname "$0")
${my_dir}/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fret sitl.py model:=scara scenario:=static_reach
