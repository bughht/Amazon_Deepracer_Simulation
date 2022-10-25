cmake_minimum_required(VERSION 2.8.3)

project(raceworld)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/camera/cam_raw.py
  scripts/camera/openvino_node.py
  scripts/camera/qrtag_node.py
  scripts/control/key_op.py
  scripts/control/Key_Cam.py
  scripts/control/Auto_Cam.py
  scripts/control/servo_commands1.py
  scripts/control/servo_commands2.py
  scripts/control/servo_commands3.py
  scripts/control/servo_commands4.py
  scripts/control/follow.py
  scripts/control/follow_smart.py
  scripts/control/follow_switch_multi.py
  scripts/control/lane_sim.py
  scripts/control/lane.py
  scripts/control/control_servo.py
  scripts/tag_locate/tag_detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
