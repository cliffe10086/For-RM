c_cpp_properties.jason includepath添加/opt中ros对应版本的include

步骤
打开两个命令行
第一个
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node
第二个
cd */*/ws 按照路径
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run turtle_chase turtle_chase_node



