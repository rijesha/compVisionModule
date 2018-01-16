#include "cvm_argument_parser.hpp"
#include "navigational_state.hpp"
#include "location_processor.hpp"
#include "position.hpp"
#include <thread>
#include "msg_queue.hpp"

void looper(LocationProcessor * lp){
    while (true){
        lp->processImage();
    }
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
    //thread t1(looper, &lp);

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

    return -1;
}
