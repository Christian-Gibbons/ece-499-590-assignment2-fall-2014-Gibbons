ECE-499-590-Fall-2014
=====================

ECE-499-590-Fall-2014

Instructions to run:

enter d_diff_drive_robot directory:
	cd d_diff_drive_robot

start the gazebo simulation:
	./robot-view server

run the object tracking process:
	python cv_tracking.py

run PID control process:
	python robot-turn-PID.py

run the robot control process:
	python robot-view-serial.py
