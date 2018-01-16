#include "cvm_argument_parser.hpp"
#include "navigational_state.hpp"
#include "location_processor.hpp"
#include <thread>

void looper(LocationProcessor * lp){
    while (true){
        lp->processImage();
    }
}

int main(int argc, const char** argv){
    CVMArgumentParser ap(argc, argv, true, false, false, false);
    
    

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
        //Send position to Mavlink Queue
        ns = ns->returnNextState(current_position);
        desired_position = ns->computeDesiredPosition(current_position);
        count++;
    }

    return -1;
}
