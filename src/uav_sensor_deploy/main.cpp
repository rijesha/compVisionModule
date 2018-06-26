#include <common/cvm_argument_parser.hpp>
#include <common/navigational_state.hpp>
#include <common/location_processor.hpp>
#include <common/position.hpp>
#include <thread>
#include <common/msg_queue.hpp>
#include <mavlink-interface/position_controller.h>
#include <mavlink-interface/multithreaded_interface.h>

bool shutdown = false;

int main(int argc, const char** argv){
    CVMArgumentParser ap(argc, argv, true, false, false, false);

    LocationProcessor lp = LocationProcessor(ap.calib_file_path, ap.deviceID);
    NavigationalState *ns = new AutoPilotState(); 
    
    Multithreaded_Interface mti;
    mti.start("/dev/ttyUSB0", 57600);

    Position_Controller pc(&mti);
    ns->pc = &pc;
    
    int count = 0;
 
    Position current_position;
    Position desired_position;
    while (count < 8){
        current_position = lp.processImage();
        pc.update_current_position(current_position.x, current_position.y, current_position.z, current_position.azi * 3.14/180);

        ns = ns->returnNextState(current_position);
        
        desired_position = ns->computeDesiredPosition(current_position);
        pc.update_desired_position(desired_position.x, desired_position.y, desired_position.z, desired_position.azi * 3.14/180);

        count++;
    }
    mti.shutdown();
    return -1;
}
