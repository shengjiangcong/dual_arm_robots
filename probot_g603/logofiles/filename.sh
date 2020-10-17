rob_name="anno"
cd ~/probot_${rob_name}_ws
source devel/setup.bash
export LD_LIBRARY_PATH=~/probot_${rob_name}_ws/src/probot_${rob_name}/probot_rviz_plugin/lib/moveIt:${LD_LIBRARY_PATH}
roslaunch probot_bringup probot_${rob_name}_bringup.launch


