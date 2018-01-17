all: mavlink_control

mavlink_control:
	g++ -I ../../libs/mavlink/ -DSTANDALONE mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread


clean:
	 rm -rf *o mavlink_control
