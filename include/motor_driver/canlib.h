#ifndef __CANLIB_H__
#define __CANLIB_H__

#include <stdlib.h>
#include <vector>
#include <math.h>
#include "ros/ros.h"

int float_to_int(float x, float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
char to_char(int data);
int to_int(char data);
void pack_cmd(char * msg, float p_des, float v_des, float kp, float kd, float t_ff);
std::vector<float> unpack_reply(const char* msg);

#endif
