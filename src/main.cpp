#include "navigational_state.hpp"
#include "location_processor.hpp"
#include <thread>

void looper(LocationProcessor * lp){
    lp->LocationProcessingThread();
}

int main(void ){

    AutoPilotState a;
    cout << a.zzz;// = 19;
    a.zzz = 19;

    cout << a.zzz;
    
    PullOutState p;
    cout << p.zzz;

    //NavigationalState n;
    //cout << n.zzz;
    p.zzz = 235;
    cout << " ";
    cout << a.zzz << endl;

    LocationProcessor lp = LocationProcessor();
    thread t1(looper, &lp);

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
