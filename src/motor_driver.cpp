#include "motor_driver/motor_driver.h"

using namespace std;

Motor::Motor(ros::NodeHandle *nodehandle){

	ros::NodeHandle nh(*nodehandle);
	ROS_INFO("Motor Driver Node Activated");
	ros::param::get("~port_name", port_name);

	serial_port = open(port_name.c_str(), O_RDWR);
	motor_state_publisher = nh.advertise<std_msgs::Float32MultiArray>("/motor/state", 1);

	ros::Subscriber state_sub = nh.subscribe("/motor/control", 1, &Motor::cmd_data_callback, this);


	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		exit(1);
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	// tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		exit(1);
	}
	write(serial_port, motor_activate_msg, 24);
	//reading();
	pack_cmd(cmd_data,0.0,0.0,0.0,0.0, 0.0);
	memset(&read_data, '\0', sizeof(read_data));
	ROS_INFO("Activate Motor\n");
	//boost::thread recieve_thread = boost::thread(boost::bind(&Motor::reading, this));

	boost::thread send_thread = boost::thread(boost::bind(&Motor::writing, this));
	ros::spin();
}

void Motor::reading(){
	// Normally you wouldn't do this memset() call, but since we will just receive
	// ASCII data for this example, we'll set everything to 0 so we can
	// call printf() easily.
	
	memset(&read_data, '\0', sizeof(read_data));
	while(ros::ok()){
		memset(&read_buf, '\0', sizeof(read_buf));
		int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
		if (num_bytes < 0) {
			ROS_ERROR("Error reading: %s", strerror(errno));
		}
		strcat(read_data, read_buf);
		
		if(strlen(read_data)>24){
			memset(&read_data, '\0', sizeof(read_data));	
			return;
		}

		printf("Read %i bytes. Received message: %s\n\n", num_bytes, read_buf);
		printf("read data : %s\n", read_data);
		if(read_buf[num_bytes-1] == '\r' && strlen(read_data)==24 ){
			ROS_INFO("Pack data to %s\n", read_data);
			vector<float> data;
			data = unpack_reply(read_data);
			std_msgs::Float32MultiArray msg;
			msg.data = data;
			motor_state_publisher.publish(msg);
			return;
		}
	}
}

void Motor::writing(){
	while (ros::ok()){
		// Write to serial port
		char send_data[30];
		strcpy(send_data, header);
		strcat(send_data, cmd_data);
		send_data[23]='\r';
		write(serial_port, send_data, 24);
		usleep(1000000/hz);
		reading();
		
		//ROS_INFO("Sent Data %s", send_data);
	}
}


void Motor::cmd_data_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	kp = msg->data[0];
	kd = msg->data[1];
	position = msg->data[2];
	velocity = msg->data[3];
	torque = msg->data[4];
	pack_cmd(cmd_data, position, velocity, kp, kd, torque);
}

Motor::~Motor(){
	write(serial_port, motor_deactivate_msg, sizeof(motor_deactivate_msg));
	ROS_INFO("Motor Deactivated!");
	close(serial_port);
	ROS_INFO("Terminated Motor driver");
}
