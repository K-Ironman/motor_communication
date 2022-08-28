#include "motor_driver/canlib.h"
float P_MIN =-12.5;
float P_MAX =12.5;
float V_MIN =-50;
float V_MAX =50;
float T_MIN =-18;
float T_MAX =18;
float Kp_MIN =0;
float Kp_MAX =500;
float Kd_MIN =0;
float Kd_MAX =5;

float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x- x_min)*((float)((1<<bits)/span)));
}
char to_char(int data){
	switch (data)
	{
	case 0:
		return '0';
	case 1:
		return '1';
	case 2:
		return '2';
	case 3:
		return '3';
	case 4:
		return '4';
	case 5:
		return '5';
	case 6:
		return '6';
	case 7:
		return '7';
	case 8:
		return '8';
	case 9:
		return '9';
	case 10:
		return 'a';
	case 11:
		return 'b';
	case 12:
		return 'c';
	case 13:
		return 'd';
	case 14:
		return 'e';
	case 15:
		return 'f';
	default:
		ROS_ERROR("wrong conversion! %d\n", data);
		abort();
	}
}
int to_int(char data){
	switch (data)
	{
	case '0':
		return 0;
	case '1':
		return 1;
	case '2':
		return 2;
	case '3':
		return 3;
	case '4':
		return 4;
	case '5':
		return 5;
	case '6':
		return 6;
	case '7':
		return 7;
	case '8':
		return 8;
	case '9':
		return 9;
	case 'a':
		return 0xa;
	case 'b':
		return 0xb;
	case 'c':
		return 0xc;
	case 'd':
		return 0xd;
	case 'e':
		return 0xe;
	case 'f':
		return 0xf;
	default:
		ROS_ERROR("wrong conversion! %d\n", data);
		abort();
	}
}

void pack_cmd(char * msg, float p_des, float v_des, float kp, float kd, float t_ff){
    /// limit data to be within bounds ///

    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    /// convert floats to unsigned ints ///
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    /// pack ints into the can buffer ///
   	msg[0] = to_char(p_int>>12); // Position 8 higher (upper 4)
    msg[1] = to_char((p_int>>8)&0xf); // Position 8 higher (lower 4)
	msg[2] = to_char((p_int>>4)&0xf); // Position 8 lower (upper 4)
	msg[3] = to_char(p_int&0xf);
	msg[4] = to_char(v_int>>8);
	msg[5] = to_char((v_int>>4)&0xf);
	msg[6] = to_char(v_int&0xf);
	msg[7] = to_char(kd_int>>8);
	msg[8] = to_char((kp_int>>4)&0xf);
	msg[9] = to_char(kp_int&0xf);
	msg[10] = to_char(kd_int>>8);
	msg[11] = to_char((kd_int>>4)&0xf);
	msg[12] = to_char(kd_int&0xf);
	msg[13] = to_char(t_int>>8);
	msg[14] = to_char((t_int>>4)&0xf);
	msg[15] = to_char(t_int&0xf);
}

std::vector<float> unpack_reply(const char* msg){
/// unpack ints from can buffer ///

	int data[8];
	for(int i=3; i<9; i++){
		//printf("%c%c\n", msg[i*2-1], msg[i*2]);
		data[i-3] = (to_int(msg[i*2-1])<<4)|to_int(msg[i*2]);
	}
	int p_int = (data[1]<<8)|data[2]; //Motor position data
	int v_int = (data[3]<<4)|(data[4]>>4); // Motor speed data
	int i_int = ((data[4]&0xF)<<8)|data[5]; // Motor torque data
	//printf("%u %u %u \n", p_int, v_int, i_int);
	/// convert ints to floats ///
	float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
	float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
	std::vector<float> return_data(3);
	return_data[0] = p;
	return_data[1] = v;
	return_data[2] = i;
	return return_data;

}
