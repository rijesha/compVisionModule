#include "cvm_argument_parser.hpp"
#include "navigational_state.hpp"
#include "location_processor.hpp"
#include <thread>

void looper(LocationProcessor * lp){
    lp->LocationProcessingThread();
}

int main(int argc, const char** argv){
    CVMArgumentParser ap(argc, argv, true, false, false);
    
    AutoPilotState a;
    
    cout << a.zzz;// = 21;
    a.zzz = 19;

    cout << a.zzz;
    
    PullOutState p;
    cout << p.zzz;

    //NavigationalState n;
    //cout << n.zzz;
    p.zzz = 235;
    cout << " ";
    cout << a.zzz << endl;

    LocationProcessor lp = LocationProcessor(ap.calib_file_path, ap.deviceID);
    thread t1(looper, &lp);

    cout << " " << endl;
    NavigationalState *ns = &a; 
    cout << ns->currentState() << " ";
    ns = ns->returnNextState();
    cout << ns->currentState() << " ";
    ns = ns->returnNextState();
    cout << ns->currentState() << " ";
    ns = ns->returnNextState();
    cout << ns->currentState() << " ";
    ns = ns->returnNextState();
    cout << ns->currentState() << " ";
    ns = ns->returnNextState();
    cout << ns->currentState() << " ";
    ns = ns->returnNextState();
    cout << ns->currentState() << " ";
    ns = ns->returnNextState();
    
    cout << ns->zzz << endl;

    int count = 0;
    while (count < 8){
        if (count == 5)
            lp.Stop();
        this_thread::sleep_for(chrono::milliseconds(1000));
        count ++;
    }
    t1.join();

    return -1;
}
