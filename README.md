## Obstacle avoiding motion planning real-time performance
Demo video is here: (https://youtu.be/-srb1WkSmKg)

## Instructions on how to connect to the robot
### From the PC side:
Add 
~~~
<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find abb_irb1200_support)/urdf/irb1200_7_70.xacro'" /> 
~~~
to the "robot_interface.launch" file.   
Run
~~~
roslaunch abb_driver robot_interface.launch robot_ip:=192.168.1.50
~~~

Or alternatively, run the robot through moveit!:  
Change
~~~
<include file="$(find abb_irb1200_support)/launch/robot_interface_streaming_irb1200_7_70.launch" >
~~~
in the file "moveit_planning_execution.launch"  
to
~~~
include file="$(find abb_irb1200_support)/launch/robot_interface_download_irb1200_7_70.launch" >
~~~  
Change "arg sim from true to false"   
Run
~~~
roslaunch abb_irb1200_7_70_moveit_config moveit_planning_execution.launch robot_ip:=192.168.1.50
~~~

### From the robot side:
Follow this tutorial (http://wiki.ros.org/abb/Tutorials)
