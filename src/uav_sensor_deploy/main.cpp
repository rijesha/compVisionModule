#include <common/cvm_argument_parser.hpp>
#include <common/navigational_state.hpp>
#include <common/location_processor.hpp>
#include <common/position.hpp>
#include <thread>
#include <common/msg_queue.hpp>
#include <mavlink-interface/position_controller.h>
#include <mavlink-interface/multithreaded_interface.h>
#include "states.hpp"

bool shutdown = false;

NavigationalState<State> * ap = new AutoPilotState();
NavigationalState<State> * ia = new InitialApproach();
NavigationalState<State> * fa = new FinalApproach();
NavigationalState<State> * da = new DataAcquisitionState();
NavigationalState<State> * po = new PullOutState();
Position_Controller * pc;

int main(int argc, const char** argv){
    CVMArgumentParser argparse(argc, argv, true, false, false, false);

    LocationProcessor lp = LocationProcessor(argparse.calib_file_path, argparse.deviceID);
    NavigationalState<State> *ns = ap; 
    
    Multithreaded_Interface mti;
    mti.start("/dev/ttyUSB0", 57600);

    pc = new Position_Controller(&mti);
    
    int count = 0;
 
    Position current_position;
    Position desired_position;
    while (count < 8){
        current_position = lp.processImage();
        pc->update_current_position(current_position.x, current_position.y, current_position.z, current_position.azi * 3.14/180);

        ns = ns->returnNextState(current_position);
        
        desired_position = ns->computeDesiredPosition(current_position);
        pc->update_desired_position(desired_position.x, desired_position.y, desired_position.z, desired_position.azi * 3.14/180);

        count++;
    }
    mti.shutdown();
    return -1;
}
