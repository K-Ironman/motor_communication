/*******************************************************
 * Copyright (C) 2020-2021 김민종(Minjong Kim) <tlemsl@dgist.ac.kr or tlemsl8963@gmail.com>
 * Version 1.0.0
 * This file is part of DGIST ARTIV control planneig team(https://github.com/DGIST-ARTIV/Control-Planning).
 *
 * DGIST ARTIV can not be copied and/or distributed without the express
 * permission of 김민종(Minjong Kim)
 * For a temporary license for autonomous vehicle
 *******************************************************/
#include "motor_driver/motor_driver.h"
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "motor_driver");
	ros::NodeHandle nh;
	Motor motor(&nh);
	ros::spin();
	ROS_INFO("Exit Motor driver good bye");
	return 0;
}
