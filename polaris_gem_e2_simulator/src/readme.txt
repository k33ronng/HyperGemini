hardcode_para.py and diagonal.py are hardcode version of the parking. Adjusting the threshold should just work.

diagonal_no_hardcode.py: use rviz to determine where to go by 2D navi function. Need to make sure fixed frame is the odom frame.
steps: run the roslaunch, run odom_pub to publish odometer data; run odom_boardcast to publish the transformation for the odometry. 
with FASTLIO: make sure odom is published. 


lane_detection1: added right lane detection to be more accurate at lane detect.
