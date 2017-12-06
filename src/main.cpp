#include "navigational_state.hpp"

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

    return -1;
}