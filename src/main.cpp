#include "cvm_argument_parser.hpp"
#include "navigational_state.hpp"
#include "location_processor.hpp"
#include "position.hpp"
#include <thread>
#include "msg_queue.hpp"
#include "mavlink_control/mavlink_control.h"

bool shutdown = false;

void looper(MessageQueue<Position> *mq){
    //char *uart_name = (char*)"/dev/ttyUSB0";
    initialize_mavlink((char*)"/dev/ttyUSB0", 57600);
    api->enable_offboard_control();
    mavlink_set_position_target_local_ned_t sp;
    mavlink_vision_position_estimate_t cp;
    
    while (!shutdown){
        Position p = mq->pop();
        if (p.isDesiredPosition){
            sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
            sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;
	        sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
        
	        sp.x   = p.x;
	        sp.y   = p.y;
	        sp.z   = p.depth;
            sp.yaw  = p.azi * 3.14/180;
            api->update_setpoint(sp);
        }
        else{
            cp.usec = get_time_usec();
            cp.x = p.x;
	        cp.y = p.y;
	        cp.z = p.depth;
            cp.yaw = p.azi;
            api->update_position(cp);
        }
    }
    api->disable_offboard_control();
    release_mavlink();
}

int main(int argc, const char** argv){
    CVMArgumentParser ap(argc, argv, true, false, false, false);
    
    MessageQueue<Position> *mq = new MessageQueue<Position>();
    cout <<"Hello mq";
    Position p1(23,23,1,12);
    cout <<"Hello post" << endl;
    mq->push(p1);
    cout << mq->pop().getBasicString() << endl;;
    

    LocationProcessor lp = LocationProcessor(ap.calib_file_path, ap.deviceID);
    thread t1(looper, mq);

    cout << " " << endl;
    NavigationalState *ns = new AutoPilotState(); 
    cout << ns->currentState() << " ";
    ns = ns->returnNextState(Position());
    cout << ns->currentState() << " ";
    ns = ns->returnNextState(Position());
    cout << ns->currentState() << " ";
    ns = ns->returnNextState(Position());
    cout << ns->currentState() << " ";
    ns = ns->returnNextState(Position());
    cout << ns->currentState() << " ";
    ns = ns->returnNextState(Position());
    cout << ns->currentState() << " ";
    ns = ns->returnNextState(Position());
    cout << ns->currentState() << " ";
    ns = ns->returnNextState(Position());
    
    
    int count = 0;
    Position current_position;
    Position desired_position;
    while (count < 8){
        current_position = lp.processImage();
        mq->push(current_position);
        ns = ns->returnNextState(current_position);
        desired_position = ns->computeDesiredPosition(current_position);
        mq->push(desired_position);
        count++;
    }
    shutdown = true;
    t1.join();

    return -1;
}
