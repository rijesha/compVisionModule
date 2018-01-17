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
    while (!shutdown){
        mq->pop();
    }
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
